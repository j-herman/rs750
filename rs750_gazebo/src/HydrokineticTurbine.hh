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

#ifndef GZ_SIM_SYSTEMS_HYDROKINETICTURBINE_HH_
#define GZ_SIM_SYSTEMS_HYDROKINETICTURBINE_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
namespace systems
{
  // Forward declaration
  class HydrokineticTurbinePrivate;

  /// \brief 
  /// TODO

  /// This plugin requires the following SDF parameters:
  /// * Required parameters:
  /// <link_name>
  ///
  /// * Optional parameters:
  /// <ges_params>
  ///    <turbine_efficiency>
  ///    <turbine_size>
  /// <topic>
  /// Example:
  //    <include>
  //      <pose>50 50 0 0 0 1.57079632</pose>
  //      <uri>model://rs750</uri>
  //      <plugin
  //        filename="libHydrokineticTurbine.so"
  //        name="gz::sim::systems::HydrokineticTurbine">
  //        <link_name>base_link</link_name>
  //        <topic>/ges_connect_topic</topic>
  //      </plugin>
  //    </include>

  class HydrokineticTurbine
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: HydrokineticTurbine();

    /// \brief Destructor
    public: ~HydrokineticTurbine() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrokineticTurbinePrivate> dataPtr;
  };
  }
}
}

#endif