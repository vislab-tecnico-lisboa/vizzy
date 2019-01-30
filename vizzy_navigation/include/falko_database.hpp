//This is an helper set of classes to save and load the model to/from a yaml file

#ifndef FALKO_DATABASE_HPP_
#define FALKO_DATABASE_HPP_

#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKO.h>

class FalkoBSCKeypoint
{
    public:
        FalkoBSCKeypoint() : descriptor(0.0, 0, 0){};
        std::string name;
        falkolib::FALKO keypoint;
        falkolib::BSC descriptor;
};


class FalkoYamlKeypoint{
    public:
        int index;
        double orientation;
        double x;
        double y;
        double radius;
};

class BCSYamlDescriptor{
    public:
        std::vector<std::vector <u_int8_t> > grid;
        int circularSectorNumber;
        int radialRingNumber;
        double sectorResolution;
        double ringResolution;
        double radius;
};

class FalkoBSCYaml
{
    public:
        FalkoYamlKeypoint keypoint;
        BCSYamlDescriptor descriptor;


};

class KeypointDatabase
{
    std::vector<FalkoBSCYaml> keypointList;
};


#endif