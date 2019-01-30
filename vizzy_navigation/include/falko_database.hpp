/*Copyright 2019, Joao Avelino, All rights reserved.*/

//This is an helper set of classes to save and load the model to/from a yaml file

#ifndef FALKO_DATABASE_HPP_
#define FALKO_DATABASE_HPP_

#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKO.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class FalkoBSCKeypoint
{
    public:
        FalkoBSCKeypoint() : descriptor(0.0, 0, 0){};
        std::string name;
        falkolib::FALKO keypoint;
        falkolib::BSC descriptor;
};

void saveModel(std::string& config_file, std::vector<FalkoBSCKeypoint>& model);
void loadModel(std::string& config_file, std::vector<falkolib::FALKO>& keypoints, std::vector<falkolib::BSC>& descriptors);

YAML::Emitter& operator<< (YAML::Emitter& out, const FalkoBSCKeypoint& v);



#endif