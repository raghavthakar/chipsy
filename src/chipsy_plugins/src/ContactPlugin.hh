#ifndef GAZEBO_PLUGINS_CONTACTPLUGIN_HH_
#define GAZEBO_PLUGINS_CONTACTPLUGIN_HH_

#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief A plugin for a contact sensor. Inherit from this class to make
  /// your own contact plugin.
  class GAZEBO_VISIBLE ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's update signal.
    /// Override this this function to get callbacks when the contact sensor
    /// is updated with new data.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
