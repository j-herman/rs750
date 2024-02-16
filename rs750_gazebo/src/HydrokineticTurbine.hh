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
  /// 
  ///
  /// * Optional parameters:
  /// 
  ///
  /// Example:
  // <plugin>
  // </plugin>
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

    /// \brief Check if an entity is enabled or not.
    /// \param[in] _entity Target entity
    /// \param[in] _ecm Entity component manager
    /// \return True if buoyancy should be applied.
    public: bool IsEnabled(Entity _entity,
        const EntityComponentManager &_ecm) const;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrokineticTurbinePrivate> dataPtr;
  };
  }
}
}

#endif