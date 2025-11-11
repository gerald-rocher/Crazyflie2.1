#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/wind.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocitySeed.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh> 
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/WindMode.hh>
#include "gz/sim/Link.hh"

#include <iostream>
#include <chrono>
#include <memory>
#include <random>

using namespace gz;
using namespace sim;
using namespace systems;

class WindPublisherSystem
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  //-------------------------
  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &_eventMgr) override
  //-------------------------
  {
      std::cout << "=== WindPublisherSystem Loaded ===" << std::endl;

      parseSdfParameters(_sdf);

      this->worldEntity = findWorldEntity(_entity, _ecm);
      if (this->worldEntity == kNullEntity) {
        std::cerr << "Could not find world entity!" << std::endl;
        return;
      }

      this->worldName = scopedName(this->worldEntity, _ecm, "::", false);
      this->windVelocityTopic = "/world/wind_velocity";
      this->windForceTopic = "/world/wind_force";

      std::cout << "World name: " << this->worldName << std::endl;
      std::cout << "Publishing wind velocities on topic: " << this->windVelocityTopic << std::endl;

      this->windVelocityPub = this->node.Advertise<msgs::Wind>(this->windVelocityTopic);
      this->windForcePub = this->node.Advertise<msgs::Wrench>(this->windForceTopic);

      // Create Wind component if it doesn't exist
      this->windEntity = _ecm.EntityByComponents(components::Wind());
      if (this->windEntity == kNullEntity) {
        this->windEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(this->windEntity, components::Wind());
      }

      auto world_linear_velocity = _ecm.Component<components::WorldLinearVelocity>(this->windEntity);
      if (!world_linear_velocity) {
        _ecm.CreateComponent(this->windEntity, 
                            components::WorldLinearVelocity(math::Vector3d::Zero));
      }

      auto world_linear_velocity_seed = _ecm.Component<components::WorldLinearVelocitySeed>(this->windEntity);
      if (!world_linear_velocity_seed) {
        _ecm.CreateComponent(this->windEntity, 
                            components::WorldLinearVelocitySeed(math::Vector3d::Zero));
      }
  }

  //-------------------------
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  //-------------------------
  {
      if (this->worldEntity == kNullEntity || _info.paused) {
        return;
      }

      auto windVelocity = getWindVelocity(_ecm);  
      
      auto world_linear_velocity_seed = _ecm.Component<components::WorldLinearVelocitySeed>(this->windEntity);
      if (world_linear_velocity_seed) {
        world_linear_velocity_seed->Data() = windVelocity;
      }

      auto world_linear_velocity = _ecm.Component<components::WorldLinearVelocity>(this->windEntity);
      if (world_linear_velocity) {
        world_linear_velocity->Data() = windVelocity;
      }

      math::Vector3d link_force = applyWindForces(windVelocity, _ecm);

      // Publish values on gz topics
      gz::msgs::Header header;
      auto *stamp = header.mutable_stamp();
      stamp->set_sec(std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
      stamp->set_nsec(std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count() % 1000000000);

      auto *frame_data = header.add_data();
      frame_data->set_key("frame_id");
      frame_data->add_value(this->frameId);

      msgs::Wind windMsg;
      windMsg.mutable_header()->CopyFrom(header);
      windMsg.mutable_linear_velocity()->set_x(windVelocity.X());
      windMsg.mutable_linear_velocity()->set_y(windVelocity.Y());
      windMsg.mutable_linear_velocity()->set_z(windVelocity.Z());

      this->windVelocityPub.Publish(windMsg);

      if (frameCounter++ % 1000 == 0) {
        gz::msgs::Wrench forceMsg;
        forceMsg.mutable_header()->CopyFrom(header);
        forceMsg.mutable_force()->set_x(link_force.X());
        forceMsg.mutable_force()->set_y(link_force.Y());
        forceMsg.mutable_force()->set_z(link_force.Z());
        forceMsg.mutable_torque()->set_x(0.0);
        forceMsg.mutable_torque()->set_y(0.0);
        forceMsg.mutable_torque()->set_z(0.0);
      
        this->windForcePub.Publish(forceMsg);
      }
  }

private:
  //-------------------------
  void parseSdfParameters(const std::shared_ptr<const sdf::Element> &_sdf)
  //------------------------- 
  {
    this->frameId                 = "";
    this->baseWindX               = 0.01; // Constant mean wind in the X direction (forward/backward)
    this->baseWindY               = 0.0;  // Constant mean wind in the Y direction (left/right)
    this->baseWindZ               = 0.0;  // Constant mean wind in the Z direction (up/down)

    this->slowAmplitude           = 0.10; // Slow gusts: sinusoidal variation of ±0.1 m/s every 30 seconds
    this->slowPeriod              = 30.0;

    this->fastAmplitude           = 0.05; // Fast gusts: sinusoidal variation of ±0.05 m/s every 5 seconds
    this->fastPeriod              = 5.0;

    this->lateralAmplitude        = 0.05; // Crosswind: side-to-side wind variation in Y direction of ±0.05 m/s every 15 seconds
    this->lateralPeriod           = 15.0;

    this->verticalAmplitude       = 0.01; // Updrafts/downdrafts: very weak vertical wind variation to remain realistic
    this->verticalPeriod          = 10.0;

    this->noiseStdDev             = 0.05; // Random noise added to wind. Higher values = more unpredictable, turbulent wind.
    this->aerodynamicCoefficient  = 0.10; // Intensity of wind forces applied to the drone. Higher values = stronger wind effects. For nano drones, keep between 0.05 and 0.2.

    if (_sdf->HasElement("frame_id")) {
      this->frameId = _sdf->Get<std::string>("frame_id", this->frameId).first;
    }
    if (_sdf->HasElement("base_wind_x")) {
      this->baseWindX = _sdf->Get<double>("base_wind_x", this->baseWindX).first;
    }
    if (_sdf->HasElement("base_wind_y")) {
      this->baseWindY = _sdf->Get<double>("base_wind_y", this->baseWindY).first;
    }
    if (_sdf->HasElement("base_wind_z")) {
      this->baseWindZ = _sdf->Get<double>("base_wind_z", this->baseWindZ).first;
    }
    if (_sdf->HasElement("slow_amplitude")) {
      this->slowAmplitude = _sdf->Get<double>("slow_amplitude", this->slowAmplitude).first;
    }
    if (_sdf->HasElement("slow_period")) {
      this->slowPeriod = _sdf->Get<double>("slow_period", this->slowPeriod).first;
    }
    if (_sdf->HasElement("fast_amplitude")) {
      this->fastAmplitude = _sdf->Get<double>("fast_amplitude", this->fastAmplitude).first;
    }
    if (_sdf->HasElement("fast_period")) {
      this->fastPeriod = _sdf->Get<double>("fast_period", this->fastPeriod).first;
    }
    if (_sdf->HasElement("lateral_amplitude")) {
      this->lateralAmplitude = _sdf->Get<double>("lateral_amplitude", this->lateralAmplitude).first;
    }
    if (_sdf->HasElement("lateral_period")) {
      this->lateralPeriod = _sdf->Get<double>("lateral_period", this->lateralPeriod).first;
    }
    if (_sdf->HasElement("vertical_amplitude")) {
      this->verticalAmplitude = _sdf->Get<double>("vertical_amplitude", this->verticalAmplitude).first;
    }
    if (_sdf->HasElement("vertical_period")) {
      this->verticalPeriod = _sdf->Get<double>("vertical_period", this->verticalPeriod).first;
    }
    if (_sdf->HasElement("noise_stddev")) {
      this->noiseStdDev = _sdf->Get<double>("noise_stddev", this->noiseStdDev).first;
    }
    if (_sdf->HasElement("aerodynamic_coefficient")) {
      this->aerodynamicCoefficient = _sdf->Get<double>("aerodynamic_coefficient", this->aerodynamicCoefficient).first;
    }
  }

  //-------------------------
  math::Vector3d getWindVelocity(EntityComponentManager &_ecm)
  //------------------------- 
  {
    static auto startTime = std::chrono::steady_clock::now();
    static std::default_random_engine gen{std::random_device{}()};
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - startTime).count();
    
    std::normal_distribution<double> dist(0.0, this->noiseStdDev);
    double noise = dist(gen);
    
    double windX = this->baseWindX + 
                   this->slowAmplitude * std::sin(2 * M_PI * elapsed / this->slowPeriod) +
                   this->fastAmplitude * std::sin(2 * M_PI * elapsed / this->fastPeriod) + 
                   noise;
    
    double windY = this->baseWindY +
                   this->lateralAmplitude * std::sin(2 * M_PI * elapsed / this->lateralPeriod) + 
                   dist(gen) * 0.1;
    
    double windZ = this->baseWindZ +
                   this->verticalAmplitude * std::sin(2 * M_PI * elapsed / this->verticalPeriod);

    return math::Vector3d(windX, windY, windZ);
  }

  //-------------------------
  math::Vector3d applyWindForces(const math::Vector3d &_windVel, EntityComponentManager &_ecm)
  //-------------------------
  {
    math::Vector3d linkForce = math::Vector3d::Zero;
    static bool foundMatch = false;

    _ecm.Each<components::Link, components::Inertial, components::WorldPose, components::WorldLinearVelocity, components::Name>(
      [&](const Entity &_entity, 
          components::Link *, 
          components::Inertial *_inertial,
          components::WorldPose *_pose,
          components::WorldLinearVelocity *_linkVel,
          components::Name *_name) -> bool
      {

        if (_name->Data() == this->frameId)
        {
          math::Vector3d relativeVel = _windVel - _linkVel->Data();
          math::Vector3d windForce;
          
          windForce.X(_inertial->Data().MassMatrix().Mass() * this->aerodynamicCoefficient * relativeVel.X());
          windForce.Y(_inertial->Data().MassMatrix().Mass() * this->aerodynamicCoefficient * relativeVel.Y());
          windForce.Z(_inertial->Data().MassMatrix().Mass() * this->aerodynamicCoefficient * relativeVel.Z());
          
          linkForce = windForce;
          
          Link link(_entity);
          link.AddWorldForce(_ecm, windForce);
          foundMatch = true;
        }        
        return true;
      });

      if (!foundMatch) {
         std::cout << " Link not found (" << this->frameId << "). Please check plugin parameters." << std::endl;
      }

      return linkForce; 
  }

  //-------------------------
  Entity findWorldEntity(Entity _startEntity, EntityComponentManager &_ecm)
  //-------------------------
  {
    Entity current = _startEntity;
    while (current != kNullEntity)
    {
      World world(current);
      if (world.Valid(_ecm))
        return current;
      current = _ecm.ParentEntity(current);
    }
    return kNullEntity;
  }

  gz::transport::Node             node;
  Entity                          worldEntity{kNullEntity};
  Entity                          windEntity{kNullEntity};
  std::string                     worldName;
  gz::transport::Node::Publisher  windVelocityPub;
  std::string                     windVelocityTopic;
  gz::transport::Node::Publisher  windForcePub;
  std::string                     windForceTopic;  
  int                             frameCounter = 0;

  std::string frameId{""};
  double baseWindX{0.01};
  double baseWindY{0.0};
  double baseWindZ{0.0};
  double slowAmplitude{0.1};
  double slowPeriod{30.0};
  double fastAmplitude{0.05};
  double fastPeriod{5.0};
  double lateralAmplitude{0.05};
  double lateralPeriod{15.0};
  double verticalAmplitude{0.01};
  double verticalPeriod{10.0};
  double noiseStdDev{0.05};
  double aerodynamicCoefficient{0.1};
};

GZ_ADD_PLUGIN(
    WindPublisherSystem,
    System,
    WindPublisherSystem::ISystemConfigure,
    WindPublisherSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WindPublisherSystem, "wind_publisher_plugin::WindPublisherSystem")


