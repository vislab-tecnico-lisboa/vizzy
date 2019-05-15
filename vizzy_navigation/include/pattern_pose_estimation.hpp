#ifndef PATTERN_POSE_ESTIMATION_HPP_
#define PATTERN_POSE_ESTIMATION_HPP_

//#include <pcl/features/ppf.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/registration/icp.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


using namespace pcl;
using namespace std;

class PatternPoseEstimation
{
	vector<pcl::PPFHashMapSearch::Ptr> hashmap_search_vector;
	double distance_threshold;
	double rot_thresh;
	double tran_thresh;
	double cluster_tran_thresh;
	double cluster_rot_thresh;
	double fitting_score_thresh;
	double discretization_step;
	pcl::PointCloud<pcl::Normal>::Ptr normals_;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	public:
		PointCloud<PointNormal>::Ptr cloud_output_subsampled;

		pcl::PointCloud<PointNormal>::Ptr getPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

		std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> dense_cloud_models;
		std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normals;

		PatternPoseEstimation(	double rot_thresh_=30.0, 
					double tran_thresh_=0.05,
					double fitting_score_thresh_=0.01, 
					double discretization_step_=0.01,
					std::string file_="file",
                                        double distance_threshold_=1.4);

		int train (std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normals_);
		Eigen::Affine3d detect(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

		Eigen::Matrix4f refine(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target);
		void loadModelFromMesh(std::string file_name="filename");
};

#endif
