/*
 * vizzyFwd.h
 *
 *  Created on: Dec 27, 2011
 *      Author: vislab
 */

#ifndef VIZZYFWD_H_
#define VIZZYFWD_H_

#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <cmath>

#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <iCub/iKin/iKinFwd.h>

#include <string>
#include <deque>

namespace iCub
{

namespace iKin
{



/**
* \ingroup iKinFwd
*
* A class for defining the 5-DOF Vizzy Eye
*/
class vizzyEye : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type, const std::string &_root_link);
    std::string root_link;
public:
    /**
    * Default constructor.
    */
    vizzyEye();

    /**
    * Constructor.
    * @param _type is a string to discriminate between "left" and
    *              "right" eye. Further available options are
    *              "left_v2" and "right_v2".
    */
    vizzyEye(const std::string &_type);

    /**
    * Constructor.
    * @param _type is a string to discriminate between "left" and
    *              "right" eye. Further available options are
    *              "left_v2" and "right_v2".
    */
    vizzyEye(const std::string &_type,const std::string &_root_link);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    vizzyEye(const vizzyEye &eye);

    /**
    * Alignes the Eye joints bounds with current values set aboard
    * the Vizzy.
    * @param lim is the ordered list of control interfaces that
    *            allows to access the Torso and the Head limits.
    * @return true/false on success/failure.
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
};


/**
* \ingroup iKinFwd
*
* A class for defining the 5-DOF Vizzy Eye with the root
* reference frame attached to the neck.
*/
class vizzyEyeNeckRef : public vizzyEye
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor.
    */
    vizzyEyeNeckRef();

    /**
    * Constructor.
    * @param _type is a string to discriminate between "left" and
    *              "right" eye. Further available options are
    *              "left_v2" and "right_v2".
    */
    vizzyEyeNeckRef(const std::string &_type);
    
    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    vizzyEyeNeckRef(const vizzyEyeNeckRef &eye);
};


class vizzyHeadCenter : public iKinLimb
{
protected:
    void allocate(const string &_type, const string &_root_link);

public:
    vizzyHeadCenter()                            { allocate("right","waist"); }
    vizzyHeadCenter(const string &_type,const string &_root_link) { allocate(_type,_root_link);   }
    vizzyHeadCenter(const vizzyHeadCenter &head) { clone(head);       }
};

/**
* \ingroup iKinFwd
*
* A class for defining the 6-DOF Inertia Sensor Kinematics
*/
class vizzyInertialSensor : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type, const std::string &_root_link);
    std::string root_link;
public:
    /**
    * Default constructor.
    */
    vizzyInertialSensor();

    /**
    * Constructor.
    * @param _type is a string to discriminate between "v1" and "v2"
    *              hardware versions.
    */
    vizzyInertialSensor(const std::string &_type);

    /**
    * Constructor.
    * @param _type is a string to discriminate between "v1" and "v2"
    *              hardware versions.
    */
    vizzyInertialSensor(const std::string &_type, const std::string &_root_link);

    /**
    * Creates a new Inertial Sensor from an already existing object.
    * @param sensor is the object to be copied.
    */
    vizzyInertialSensor(const vizzyInertialSensor &sensor);

    /**
    * Alignes the Inertial Sensor joints bounds with current values
    * set aboard the Vizzy.
    * @param lim is the ordered list of control interfaces that
    *            allows to access the Torso and the Head limits.
    * @return true/false on success/failure.
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
};

}

}

#endif /* VIZZYFWD_H_ */
