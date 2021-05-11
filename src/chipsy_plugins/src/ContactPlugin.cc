#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
  gzdbg<<"Brooo\n";
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Uncoment the following lines for debug output
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      std::cout << "   Normal force 1: "
                << contacts.contact(i).normal(j).x() *
                   contacts.contact(i).wrench(j).body_1_wrench().force().x() +
                   contacts.contact(i).normal(j).y() *
                   contacts.contact(i).wrench(j).body_1_wrench().force().y() +
                   contacts.contact(i).normal(j).z() *
                   contacts.contact(i).wrench(j).body_1_wrench().force().z()
                   << "\n";
    }
  }
  gzdbg<<"Brooo\n";
}
