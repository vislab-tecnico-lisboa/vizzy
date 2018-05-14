/*!
    \file mimic_plugin.hh
    \brief Gazebo mimic plugin
    
    Gazebo plugin for joint mimicking behaviour

    \author João Borrego : jsbruglie
*/

#ifndef _MIMIC_PLUGIN_HH_
#define _MIMIC_PLUGIN_HH_

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace MimicPlugin {

    // Plugin parameters instanced in SDF

    /// Name SDF attribute
    #define PARAM_NAME              "name"
    /// Mimic joint multiplier SDF attribute
    #define PARAM_MULTIPLIER        "multiplier"
    /// Finger joint name SDF entity
    #define PARAM_JOINT_GROUP       "actuatedJoint"
    /// Mimic joint name SDF entity
    #define PARAM_MIMIC_JOINT       "mimicJoint"
}

namespace gazebo {

    // Forward declaration of private joint group class
    class JointGroup;

    // Forward declaration of private data class
    class MimicPluginPrivate;

    /// \brief gazebo mimic plugin
    class MimicPlugin : public ModelPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<MimicPluginPrivate> data_ptr;

        // Public methods

        /// \brief Constructs the object
        public: MimicPlugin();

        /// \brief Destroys the object
        public: virtual ~MimicPlugin();

        /// \brief Loads the plugin
        /// \param _model The model pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Updates model on world update
        public: void onUpdate();

        // Private methods

        /// \brief Loads mimic joints in a single group
        /// \param _sdf     The sdf element pointer
        /// \param group    The joint group
        /// \returns Success
        private: bool loadMimicJoints(sdf::ElementPtr _sdf, JointGroup & group);

        /// \brief Loads joint groups
        /// \param _sdf The root sdf element pointer
        /// \returns Success
        private: bool loadJointGroups(sdf::ElementPtr _sdf);
    };
}

#endif