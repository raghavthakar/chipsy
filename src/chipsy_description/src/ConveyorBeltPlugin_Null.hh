#ifndef _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_
#define _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Angle.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE ConveyorBeltPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ConveyorBeltPlugin() = default;

    /// \brief Destructor.
    public: virtual ~ConveyorBeltPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate();

    /// \brief Get if the belt is enabled.
    /// \return True if enabled.
    protected: bool IsEnabled() const;

    /// \brief Get the power of the conveyor belt.
    /// \return Power of the belt as a percentage (0-100).
    protected: double Power() const;

    /// \brief Set the power of the conveyor belt.
    /// \param[in] _power Power of the belt as a percentage (0-100).
    protected: void SetPower(const double _power);

    /// \brief Overwrite this method for sending periodic updates with the
    /// conveyor state.
    private: virtual void Publish() const;

    /// \brief Call back for enable/disable messaged.
    protected: void OnEnabled(ConstGzStringPtr &_msg);

    /// \brief Belt velocity (m/s).
    protected: double beltVelocity = 0.0;

    /// \brief Belt power expressed as a percentage of the internal maximum
    /// speed.
    protected: double beltPower = 0.0;

    /// \brief If true, power commands are processed, otherwise the belt won't move.
    protected: bool enabled = true;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief The joint that controls the movement of the belt.
    private: physics::JointPtr joint;

    /// \brief The belt's link.
    private: physics::LinkPtr link;

    /// \brief When the joint reaches this point, it will go back to its initial
    /// position.
    private: double limit;

    /// \brief Maximum linear velocity of the belt.
    private: const double kMaxBeltLinVel = 0.2;

    /// \brief Gazebo node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo publisher for modifying the rate of populating the belt.
    public: transport::PublisherPtr populationRateModifierPub;

    /// \brief Gazebo subscriber for modifying the enabled state of the belt.
    public: transport::SubscriberPtr enabledSub;
  };
}
#endif
