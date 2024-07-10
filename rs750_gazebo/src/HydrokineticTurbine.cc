// Copyright (C) 2024 Jessica Herman
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// This code modified from the gazebo LiftDrag plugin
/*
* Copyright (C) 2012 Open Source Robotics Foundation
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*/

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/float.pb.h>

#include <mutex>
#include <string>
#include <vector>

//#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>
#include <sdf/sdf.hh>

#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "HydrokineticTurbine.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::HydrokineticTurbinePrivate
{
  /// \brief Initialize the plugin.
  /// \param[in] _ecm Immutable reference to the EntityComponentManager.
  /// \param[in] _sdf The SDF Element associated with this system plugin.
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Callback to pause/resume battery charging.
  /// \param[in] _paused Set to true when the battery charging is disabled
  /// due to connection with mothership for battery swap.  Set to false for
  /// normal operations.
  public: void OnConnect(const msgs::Boolean &_paused);

  /// \brief Callback to update axial induction factor.
  /// \param[in] _zeta Set to desired new value.
  public: void OnZeta(const msgs::Float &_zeta);

  /// \brief A mutex to protect variable updates
  public: std::mutex mutex;

  /// \brief Gazebo transport node
  public: transport::Node node;

  /// \brief Topic for stopping and restarting charging
  public: std::string charge_topic = "/ges/charge";

  /// \brief Topic for adjusting axial induction factor
  public: std::string zeta_topic = "/ges/zeta";

  /// \brief Topic for publishing power generation rate
  public: std::string topicPwrGen = "/ges/wattage";

  /// \brief Topic for publishing turbine drag
  public: std::string topicTrbnDrag = "/ges/drag";

  /// \brief Transport node publisher for power generation
  public: transport::Node::Publisher pwrGenPub;

  /// \brief Transport node publisher for turbine drag
  public: transport::Node::Publisher trbnDragPub;

  /// \brief Update rate buffer for power publisher
  public: double updateRate = 10;

  /// \brief The link interface representing the turbine
  public: gz::sim::Link link;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The pose of the model relative to the world frame
  public: gz::math::Pose3<double> modelPose;

  /// \brief Linear force to apply at the turbine link, for scaling with velocity.  To be replaced, eventually, when calculating axial induction factor.
  public: double unitTurbineDragForce;

  /// The following turbine parameters are sourced from Watt&Sea 600

  /// \brief Efficiency parameter for turbine power generation
  public: double turbineEfficiency = 0.39;

  /// \brief Size parameter (placeholder) for turbine power generation - m^2
  public: double turbineSize = 0.0616;

  /// \brief Axis in which to measure turbine velocity/ water flow rate
  public: std::string axis = "X";

  /// \brief Turbine axial induction factor, placeholder for future calculation or optimization
  public: double zeta = 0.781;

  /// \brief Current rate (in Watts/sec?)
  public: double watts = 0;

  /// \brief Initialization flag.
  public: bool initialized{false};

  /// \brief Copy of the sdf that initializes this plugin
  public: sdf::ElementPtr sdf;

  /// \brief Bool to check/set whether the battery is charging or paused
  public: bool pause_charge = false;

  public: const double RHO_W = 1025; // density of saltwater, g/L
};

//////////////////////////////////////////////////
void HydrokineticTurbinePrivate::Load(const EntityComponentManager &_ecm,
    const sdf::ElementPtr &_sdf)
{
  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->link = Link(this->model.LinkByName(_ecm, linkName));
  if (!this->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  if (_sdf->HasElement("axis"))
  {
    this->axis = _sdf->Get<std::string>("axis");
    gzmsg << "Turbine axis set to " << this->axis << std::endl;
  }
  else 
    gzmsg << "Turbine axis not set!" << std::endl;

  if (!_sdf->HasElement("ges_params"))
  {
    gzmsg << "No parameters specified.  Using defaults." << std::endl;
  }

  else
  {
    sdf::ElementPtr paramsSDF = sdf->GetElement("ges_params");
    if (paramsSDF->HasElement("turbine_efficiency"))
    {
      this->turbineEfficiency = paramsSDF->Get<double>("turbine_efficiency");
    }
    if (paramsSDF->HasElement("turbine_size"))
    {
      this->turbineSize = paramsSDF->Get<double>("turbine_size");
    }
  }

  // Subscribe to mothership charging topic
  if (_sdf->HasElement("topic"))
    this->charge_topic = _sdf->Get<std::string>("topic");

  this->charge_topic = transport::TopicUtils::AsValidTopic(this->charge_topic);

  this->node.Subscribe(charge_topic, &HydrokineticTurbinePrivate::OnConnect, this);

  gzmsg << "HydrokineticTurbine "
      << this->model.Name(_ecm) << " subscribed to charge status messages on " 
      << this->charge_topic << std::endl;

  if (_sdf->HasElement("zeta_topic"))
    this->zeta_topic = _sdf->Get<std::string>("zeta_topic");

  this->zeta_topic = transport::TopicUtils::AsValidTopic(this->zeta_topic);

  this->node.Subscribe(zeta_topic, &HydrokineticTurbinePrivate::OnZeta, this);

  gzmsg << "HydrokineticTurbine "
      << this->model.Name(_ecm) << " subscribed to axial induction factor change messages on " 
      << this->zeta_topic << std::endl;
}

/////////////////////////////////////////////////
void HydrokineticTurbinePrivate::OnConnect(const msgs::Boolean &_charge)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->pause_charge = _charge.data();
  this->watts = 0;
}

/////////////////////////////////////////////////
void HydrokineticTurbinePrivate::OnZeta(const msgs::Float &_zeta)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->zeta = _zeta.data();
  this->unitTurbineDragForce = 0.5*(1-(this->zeta*this->zeta))*this->turbineSize*this->RHO_W;
}

//////////////////////////////////////////////////
HydrokineticTurbine::HydrokineticTurbine()
  : dataPtr(std::make_unique<HydrokineticTurbinePrivate>())
{
}

//////////////////////////////////////////////////
void HydrokineticTurbine::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->sdf = _sdf->Clone();

  // Set up publisher
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(this->dataPtr->updateRate);

  this->dataPtr->pwrGenPub = this->dataPtr->node.Advertise<msgs::Float>(
      this->dataPtr->topicPwrGen, opts);

  this->dataPtr->trbnDragPub = this->dataPtr->node.Advertise<msgs::Float>(
      this->dataPtr->topicTrbnDrag, opts);

  this->dataPtr->unitTurbineDragForce = 0.5*(1-(this->dataPtr->zeta*this->dataPtr->zeta))*this->dataPtr->turbineSize*this->dataPtr->RHO_W;
}

//////////////////////////////////////////////////
void HydrokineticTurbine::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  //gzmsg << "HydrokineticTurbine::PreUpdate" << std::endl;

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (_info.paused || this->dataPtr->pause_charge)
      return;
  }

  if (!this->dataPtr->initialized)
  {
    // Read and initialize from SDF 
    this->dataPtr->Load(_ecm, this->dataPtr->sdf);
    enableComponent<components::LinearVelocity>(_ecm, this->dataPtr->link.Entity(), true);
    enableComponent<components::WorldPose>(_ecm, this->dataPtr->link.Entity());
    this->dataPtr->link.EnableVelocityChecks(_ecm, true);
    this->dataPtr->initialized = true;
  }

  msgs::Float pwrGenMsg;
  msgs::Float trbnDragMsg;
  // Velocity of the turbine in the world frame
  gz::math::Vector3d linkVel(this->dataPtr->link.WorldLinearVelocity(_ecm).value());

  // Velocity of the turbine in its own frame
  this->dataPtr->modelPose = gz::sim::worldPose(this->dataPtr->link.Entity(), _ecm);
  gz::math::Vector3d linkVelTurbineFrame =
    this->dataPtr->modelPose.Rot().RotateVectorReverse(linkVel);
  double turbineFwdVel = 0;
  if (this->dataPtr->axis == "X")
  {
    turbineFwdVel = linkVelTurbineFrame.X();
    //gzmsg << "Turbine forward velocity X: " << turbineFwdVel << std::endl;
  }
  else if (this->dataPtr->axis == "Y")
  {
    turbineFwdVel = linkVelTurbineFrame.Y();
    //gzmsg << "Turbine forward velocity Y: " << turbineFwdVel << std::endl;
  }
  else 
  {
    gzmsg << "Turbine forward velocity error" << std::endl;
    return;
  }

  // Helper math
  double dPAt = turbineFwdVel*turbineFwdVel*this->dataPtr->unitTurbineDragForce;
  double vBar = turbineFwdVel*(1+this->dataPtr->zeta)/2;
  // Calculate and publish turbine shaft power
  double pwr = this->dataPtr->turbineEfficiency*dPAt*vBar;

  // Calculate turbine drag in boat frame
  double x, y = 0;
  if (this->dataPtr->axis == "X")
  {
    x = -dPAt;
  }
  else if (this->dataPtr->axis == "Y")
  {
    y = -dPAt;
  }
  gz::math::Vector3d drag(x,y,0);

  //Publish to Power and Drag Topic
  pwrGenMsg.set_data(pwr);
  this->dataPtr->pwrGenPub.Publish(pwrGenMsg);

  trbnDragMsg.set_data(x + y);
  this->dataPtr->trbnDragPub.Publish(trbnDragMsg);

  // Apply the drag force at link
  gz::math::Vector3d force(this->dataPtr->modelPose.Rot().RotateVector(drag));
  this->dataPtr->link.AddWorldWrench(_ecm, force, gz::math::Vector3d(0,0,0));
  //gzmsg << "zeta: " << this->dataPtr->zeta << "; unitDrag: " << this->dataPtr->unitTurbineDragForce << std::endl;
}

GZ_ADD_PLUGIN(HydrokineticTurbine,
                    gz::sim::System,
                    HydrokineticTurbine::ISystemConfigure,
                    HydrokineticTurbine::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(HydrokineticTurbine,
                          "gz::sim::systems::HydrokineticTurbine")

