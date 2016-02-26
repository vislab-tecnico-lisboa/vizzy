#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <sstream>

tf::Transform transform;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}





bool computeMatrix(PointCloud::Ptr target,
                   PointCloud::Ptr world,
                   std::string target_name,
                   std::string world_name,
                   const bool broadcast)
{
    if ((!world_name.empty()) && (!target_name.empty()) &&
            (target->points.size() > 2) && (world->points.size() == target->points.size()))
    {
        Eigen::Matrix4f trMatrix;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;

        svd.estimateRigidTransformation(*target, *world, trMatrix);

        ROS_INFO("Registration completed and Registration Matrix is being broadcasted");

        transform=tf::Transform(tf::Matrix3x3(trMatrix(0, 0), trMatrix(0, 1), trMatrix(0, 2),
                                              trMatrix(1, 0), trMatrix(1, 1), trMatrix(1, 2),
                                              trMatrix(2, 0), trMatrix(2, 1), trMatrix(2, 2)),
                                tf::Vector3(trMatrix(0, 3), trMatrix(1, 3), trMatrix(2, 3)));

        Eigen::Vector3d origin(transform.getOrigin());
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
        std::cout << std::endl << "#################################################" << std::endl;
        std::cout << std::endl << "########### TRANSFORMATION PARAMETERS ###########" << std::endl;
        std::cout << std::endl << "#################################################" << std::endl;
        std::cout << "origin: "<<origin.transpose() << std::endl;
        std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;

        Eigen::Vector3d origin_inv(transform.inverse().getOrigin());
        double roll_inv, pitch_inv, yaw_inv;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll_inv, pitch_inv, yaw_inv);
        std::cout << std::endl << "#################################################" << std::endl;
        std::cout << std::endl << "########### INVERSE TRANSFORMATION PARAMETERS ###########" << std::endl;
        std::cout << std::endl << "#################################################" << std::endl;
        std::cout << "origin: "<<origin_inv.transpose() << std::endl;
        std::cout << "rpy: " << roll_inv << " " << pitch_inv << " " << yaw_inv << std::endl;
    }

    return true;
}




int main(int argc, char *argv[])
{
    ros::init (argc, argv, "marker_detect");
    ros::NodeHandle n;

    ros::NodeHandle n_priv("~");

    int number_of_points;
    int number_of_markers;
    std::string marker_link_prefix;
    std::string first_camera_frame;
    std::string second_camera_frame;


    n_priv.param<int>("number_of_points",number_of_points, 6);
    n_priv.param<int>("number_of_markers",number_of_markers, 9);


    n_priv.param<std::string>("first_camera_frame",first_camera_frame, "first_camera_link");
    n_priv.param<std::string>("second_camera_frame",second_camera_frame, "second_camera_link");
    n_priv.param<std::string>("marker_link_prefix",marker_link_prefix, "ar_marker_");

    std::vector <std::string> marker_links;

    for(int i=0; i<number_of_points; ++i)
    {
        std::stringstream ss;
        ss << i;
        marker_links.push_back(marker_link_prefix+ss.str());
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();


    PointCloud::Ptr first_camera_cloud(new PointCloud);
    PointCloud::Ptr second_camera_cloud(new PointCloud);

    int p=0;
    do
    {
        for(int i=0; i< number_of_markers;)
        {
            tf::TransformListener listener;
            // Get some point correspondences
            try
            {
                ///////////////////////////////
                // Point in 1st camera frame //
                ///////////////////////////////

                tf::StampedTransform first_camera_tf;
                listener.waitForTransform(first_camera_frame, "first/"+marker_links[i], ros::Time(0), ros::Duration(1.0) );
                listener.lookupTransform(first_camera_frame, "first/"+marker_links[i], ros::Time(0), first_camera_tf);

                tf::StampedTransform second_camera_tf;
                listener.waitForTransform(second_camera_frame, "second/"+marker_links[i], ros::Time(0), ros::Duration(1.0) );
                listener.lookupTransform(second_camera_frame, "second/"+marker_links[i], ros::Time(0), second_camera_tf);
                ////////////////////////////////
                // Get points in camera frame //
                ////////////////////////////////

                PointT first_camera_point;
                first_camera_point.x=first_camera_tf.getOrigin().x();
                first_camera_point.y=first_camera_tf.getOrigin().y();
                first_camera_point.z=first_camera_tf.getOrigin().z();
                first_camera_cloud->points.push_back(first_camera_point);

                PointT second_camera_point;
                second_camera_point.x=second_camera_tf.getOrigin().x();
                second_camera_point.y=second_camera_tf.getOrigin().y();
                second_camera_point.z=second_camera_tf.getOrigin().z();
                second_camera_cloud->points.push_back(second_camera_point);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                continue;
            }
            ++i;
            ROS_INFO_STREAM("  Markers acquired so far: "
                            << i
                            << " out of "
                            <<number_of_markers);
        }
        ++p;
        ROS_INFO_STREAM("Samples acquired so far: "
                        << p
                        << " out of "
                        <<number_of_points);
    }
    while(p<number_of_points);

	ROS_INFO("WHA");
    computeMatrix(first_camera_cloud,
                  second_camera_cloud,
                  first_camera_frame,
                  second_camera_frame,
                  true);

    ros::Rate r(100.0);
    while(ros::ok)
    {
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                              first_camera_frame, second_camera_frame));

        r.sleep();
    }

    return 1;
}


