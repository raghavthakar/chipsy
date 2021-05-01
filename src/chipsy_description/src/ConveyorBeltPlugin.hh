#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class ConveyorBeltPlugin : public ModelPlugin
  {
  //Constructor
  public: ConveyorBeltPlugin() = default;

  //Destructor
  public: virtual ~ConveyorBeltPlugin();

  //Mandatory Load method
  public: virtual void Load(physics::ModelPtr _model,
                            sdf::ElementPtr _sdf);

  //Get information about the joint in question
  public: virtual void getInfo(physics::JointPtr joint);

  protected: void OnUpdate();

  /// \brief Pointer to the update event connection.
  private: event::ConnectionPtr updateConnection;

  //Stores name of belt joint
  private: std::string belt_joint_name;

  //Stores a shared pointer to the belt joint
  private: physics::JointPtr belt_joint;

  //Stores lower limit of actuation of the belt
  private: double belt_lower_limit;

  //Stores upper limit of actuation of the belt
  private: double belt_upper_limit;
  };
}
