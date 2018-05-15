/*!
    \file mimic_plugin.cc
    \brief Gazebo mimic plugin
    
    Gazebo plugin for joint mimicking behaviour

    \author João Borrego : jsbruglie
*/

#include <mimic_plugin/mimic_plugin.h>

namespace gazebo {

/// \brief Class for joint and respective mimics data.
class JointGroup
{
    /// Actuated joint
    public: physics::JointPtr actuated;
    /// Vector of mimic joints
    public: std::vector<physics::JointPtr> mimic;
    /// Vector of corresponding mimic joint's multipliers
    public: std::vector<double> multipliers;

    public: JointGroup(physics::JointPtr joint)
    {
        this->actuated = joint;
    }
};

/// \brief Class for private hand plugin data.
class MimicPluginPrivate
{
    /// Connection to world update event
    public: event::ConnectionPtr update_connection;
    /// Model to which the plugin is attached
    public: physics::ModelPtr model;
    /// Array of joint groups
    public: std::vector<JointGroup> groups;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MimicPlugin)

/////////////////////////////////////////////////
MimicPlugin::MimicPlugin() : ModelPlugin(),
    data_ptr(new MimicPluginPrivate)
{
    gzdbg << "[MimicPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
MimicPlugin::~MimicPlugin()
{

    gzdbg << "[MimicPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void MimicPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Validate model to which plugin is attached
    if (_model->GetJointCount() != 0) {
        this->data_ptr->model = _model;
    } else {
        gzerr << "[MimicPlugin] Invalid model." << std::endl;
        return;
    }

    // Extract parameters from SDF element

    // Finger joints
    if (loadJointGroups(_sdf) != true) return;

    // Connect to world update event
    this->data_ptr->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MimicPlugin::onUpdate, this));

    gzdbg << "[MimicPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
bool MimicPlugin::loadMimicJoints(sdf::ElementPtr _sdf, JointGroup & group)
{
    std::string mimic_name;
    double multiplier;

    if (_sdf->HasElement(PARAM_MIMIC_JOINT))
    {
        sdf::ElementPtr mimic_sdf;
        for (mimic_sdf = _sdf->GetElement(PARAM_MIMIC_JOINT);
            mimic_sdf != NULL;
            mimic_sdf = mimic_sdf->GetNextElement())
        {
            if (mimic_sdf->HasAttribute(PARAM_NAME) &&
                mimic_sdf->HasAttribute(PARAM_MULTIPLIER))
            {
                mimic_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(mimic_name);
                mimic_sdf->GetAttribute(PARAM_MULTIPLIER)->Get<double>(multiplier);
                group.mimic.push_back(this->data_ptr->model->GetJoint(mimic_name));
                group.multipliers.push_back(multiplier);

                gzdbg << "Mimic joint " << mimic_name << " k = " << multiplier << std::endl;
            }
            else if (mimic_sdf->GetName() == PARAM_MIMIC_JOINT)
            {
                gzerr << "[MimicPlugin] No joint name provided." << std::endl;
                return false;
            }
        }
    }
}


/////////////////////////////////////////////////
bool MimicPlugin::loadJointGroups(sdf::ElementPtr _sdf)
{
    std::string joint_name;
    int inserted = 0;

    if (_sdf->HasElement(PARAM_JOINT_GROUP))
    {
        sdf::ElementPtr joint_sdf;
        for (joint_sdf = _sdf->GetElement(PARAM_JOINT_GROUP);
            joint_sdf != NULL;
            joint_sdf = joint_sdf->GetNextElement())
        {
            if (joint_sdf->HasAttribute(PARAM_NAME))
            {
                joint_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(joint_name);
                this->data_ptr->groups.emplace_back(this->data_ptr->model->GetJoint(joint_name));
                gzdbg << "Actuated joint " << joint_name << std::endl;
                loadMimicJoints(joint_sdf, this->data_ptr->groups.at(inserted++));
            }
            else if (joint_sdf->GetName() == PARAM_JOINT_GROUP)
            {
                gzerr << "[MimicPlugin] No joint name provided." << std::endl;
                return false;
            }
        }
    }
    else
    {
        gzerr << "[MimicPlugin] No joints specified." << std::endl;
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
void MimicPlugin::onUpdate()
{
    for (int i = 0; i < this->data_ptr->groups.size(); i++)
    {
        physics::JointPtr actuated = this->data_ptr->groups.at(i).actuated;

        // Compatibility with Gazebo 7
        #if GAZEBO_MAJOR_VERSION < 8
            math::Angle angle = actuated->GetAngle(0).Radian();
        #else
            double position = actuated->Position();
        #endif
        
        for (int j = 0; j < this->data_ptr->groups.at(i).mimic.size(); j++)
        {
            physics::JointPtr mimic_joint = this->data_ptr->groups.at(i).mimic.at(j);
            double multiplier = this->data_ptr->groups.at(i).multipliers.at(j);
            
            #if GAZEBO_MAJOR_VERSION < 8
                mimic_joint->SetPosition(0, angle * multiplier);
            #else
                mimic_joint->SetPosition(0, position * multiplier);
            #endif
        }
    }
}

}
