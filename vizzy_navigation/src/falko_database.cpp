/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <falko_database.hpp>

#include <iostream>

YAML::Emitter& operator<< (YAML::Emitter& out, const FalkoBSCKeypoint& v)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << v.name;
    
    out << YAML::Key << "keypoint";
    out << YAML::BeginMap;
    out << YAML::Key << "index";
    out << YAML::Value << v.keypoint.index;
    out << YAML::Key << "orientation";
    out << YAML::Value << v.keypoint.orientation;
    out << YAML::Key << "x";
    out << YAML::Value << v.keypoint.point.x();
    out << YAML::Key << "y";
    out << YAML::Value << v.keypoint.point.y();
    out << YAML::Key << "radius";
    out << YAML::Value << v.keypoint.radius;
    out << YAML::EndMap;

    out << YAML::Key << "descriptor";
    out << YAML::BeginMap;
    out << YAML::Key << "grid";
    out << YAML::Value << v.descriptor.grid;
    out << YAML::Key << "circularSectorNumber";
    out << YAML::Value << v.descriptor.circularSectorNumber;
    out << YAML::Key << "radialRingNumber";
    out << YAML::Value << v.descriptor.radialRingNumber;
    out << YAML::Key << "sectorResolution";
    out << YAML::Value << v.descriptor.sectorResolution;
    out << YAML::Key << "ringResolution";
    out << YAML::Value << v.descriptor.ringResolution;
    out << YAML::Key << "radius";
    out << YAML::Value << v.descriptor.radius;
    out << YAML::EndMap;
    out << YAML::EndMap;
}


void saveModel(std::string& config_file, std::vector<FalkoBSCKeypoint>& model)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "model";
    out << YAML::Value << model;
    out << YAML::EndMap;
    std::ofstream fout(config_file);
    fout << out.c_str();


}

void loadModel(std::string& config_file, std::vector<falkolib::FALKO>& keypoints, std::vector<falkolib::BSC>& descriptors)
{
    std::cout << "Config file: " << config_file << std::endl;
    YAML::Node root = YAML::LoadFile(config_file);

    const YAML::Node& model = root["model"];

    for(auto& element : model)
    {

        std::cout << element["name"].as<std::string>() << std::endl;

        //Load keypoint
        falkolib::FALKO keypoint;
        keypoint.index = element["keypoint"]["index"].as<int>();
        keypoint.orientation = element["keypoint"]["orientation"].as<double>();
        keypoint.radius = element["keypoint"]["radius"].as<double>();
        
        double x = element["keypoint"]["x"].as<double>();
        double y = element["keypoint"]["y"].as<double>();

        keypoint.point = falkolib::Point2d(x, y);

        keypoints.push_back(keypoint);

        //Load descriptor

        double radius = element["descriptor"]["radius"].as<double>();
        int circularSectorNumber = element["descriptor"]["circularSectorNumber"].as<int>();
        int radialRingNumber = element["descriptor"]["radialRingNumber"].as<int>();
        
        falkolib::BSC descriptor(radius, circularSectorNumber, radialRingNumber);

        descriptor.sectorResolution = element["descriptor"]["sectorResolution"].as<double>();
        descriptor.ringResolution = element["descriptor"]["ringResolution"].as<double>();
        
        
        for(auto& grid : element["descriptor"]["grid"])
        {
            std::vector<u_int8_t> line;
            line = grid.as<std::vector<u_int8_t> >();
            descriptor.grid.push_back(line);
        }

        descriptors.push_back(descriptor);

    }


}