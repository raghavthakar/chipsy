#include "ConveyorBeltPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)

ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
  this->updateConnection.reset();
}

void ConveyorBeltPlugin::Load(physics::ModelPtr _model,
   sdf::ElementPtr _sdf)
  {
    if(_sdf->HasElement("belt_joint"))
      this->belt_joint_name = _sdf->GetElement("belt_joint")->Get<std::string>();

    gzdbg<<"Trying to use Joint with name: "<<this->belt_joint_name<<"\n";

    this->belt_joint=_model->GetJoint(belt_joint_name);

    if(!this->belt_joint)
    {
      gzerr<<"Did not find Joint "<<this->belt_joint_name<<"\n";
      return;
    }
    else
    {
      gzdbg<<"Found joint "<<this->belt_joint_name<<"\n";
    }

    getInfo(this->belt_joint);

    // Listen to the update event that is broadcasted every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ConveyorBeltPlugin::OnUpdate, this));
  }

  void ConveyorBeltPlugin::getInfo(physics::JointPtr joint)
  {
    this->belt_lower_limit=joint->LowerLimit();
    this->belt_upper_limit=joint->UpperLimit();
  }

  void ConveyorBeltPlugin::OnUpdate()
  {
    // gzdbg<<"In the update method. \n";
    if(belt_joint->Position()<this->belt_upper_limit)
      this->belt_joint->SetVelocity(0, 0.25);
    else
      this->belt_joint->SetPosition(0, 0, true);

    // gzdbg<<belt_joint->Position()<<"\n";
  }
