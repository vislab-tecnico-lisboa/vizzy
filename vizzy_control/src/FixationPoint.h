#ifndef FIXATIONPOINT_H
#define FIXATIONPOINT_H

#include <Eigen/Core>
#include <iostream>

class FixationPoint
{
    //double base_line;
    //double half_base_line;
public:
    FixationPoint(const double & base_line_);

    // Compute fixation point given L and R and eyes tilt
    Eigen::Vector3d getFixationPoint(const double & left_eye_angle_,
                                     const double & right_eye_angle_,
                                     const double & eyes_tilt_angle_,
                                     const double & base_line_);

};

#endif // FIXATIONPOINT_H
