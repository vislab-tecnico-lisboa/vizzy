#include <pattern_pose_estimation.hpp>
#include <random>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;

PatternPoseEstimation::PatternPoseEstimation(double rot_thresh_, double tran_thresh_, double fitting_score_thresh_, double discretization_step_, std::string file_, double distance_threshold_) :
	rot_thresh( (rot_thresh_ / 180.0) * double (M_PI)),
	tran_thresh(tran_thresh_),
	fitting_score_thresh(fitting_score_thresh_),
	normals (new pcl::PointCloud<pcl::Normal>()),
	normals_ (new pcl::PointCloud<pcl::Normal>()),
	point_cloud_ (new pcl::PointCloud<pcl::PointXYZ>()),
	tree (new pcl::search::KdTree<pcl::PointXYZ> ()),
	cloud_output_subsampled(new PointCloud<PointNormal>()),
	discretization_step(discretization_step_),
        distance_threshold(distance_threshold_)
{
        std::cout << distance_threshold << std::endl;
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	cluster_tran_thresh=3.0*tran_thresh;
	cluster_rot_thresh=3.0*rot_thresh;
	loadModelFromMesh(file_);
	train(dense_cloud_models);
}

pcl::PointCloud<PointNormal>::Ptr PatternPoseEstimation::getPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (point_cloud);
	sor.setLeafSize (discretization_step, discretization_step,1000.0);
	sor.filter (*point_cloud);

	// Define random generator with Gaussian distribution
	const double mean = 0.0;
	const double stddev = 0.001;
	std::default_random_engine generator;
	std::normal_distribution<double> dist(mean, stddev);

	for(unsigned int i=0; i<point_cloud->size();++i)
	{
		point_cloud->at(i).z=dist(generator);
	}	

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (point_cloud);
	ne.setViewPoint (std::numeric_limits<float>::max (), 0.0, 0.0);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (5.0*discretization_step);
	ne.compute (*normals);

	normals_->clear();
	point_cloud_->clear();

	for(unsigned int i=0; i<normals->size();++i)
	{
		double sqrt_=sqrt(normals->at(i).normal_x*normals->at(i).normal_x+normals->at(i).normal_y*normals->at(i).normal_y);
		normals->at(i).normal_x=normals->at(i).normal_x/sqrt_;
		normals->at(i).normal_y=normals->at(i).normal_y/sqrt_;
		normals->at(i).normal_z=0.0;
		point_cloud->at(i).z=0.0;
		if( isnan(normals->at(i).normal_x)||
		    isnan(normals->at(i).normal_y)||
		    isnan(normals->at(i).normal_z)||
		    isnan(point_cloud->at(i).x)||
		    isnan(point_cloud->at(i).y)||
		    isnan(point_cloud->at(i).z)
		  ) 
		{
			continue;
		}
		// filter also based on distance
		double distance=sqrt(point_cloud->at(i).x*point_cloud->at(i).x+point_cloud->at(i).y*point_cloud->at(i).y);

                if(distance>distance_threshold)
                   continue;
		normals_->push_back(normals->at(i));
		point_cloud_->push_back(point_cloud->at(i));
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*point_cloud_, *normals_, *cloud_normals);

	return cloud_normals;
}

void PatternPoseEstimation::loadModelFromMesh(std::string file_name)
{
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileSTL(file_name, mesh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr verts(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(mesh.cloud, *verts);

	dense_cloud_models.push_back(getPointNormal(verts));
}

int PatternPoseEstimation::train(std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normals_)
{
	PCL_INFO ("Training models ...\n");
	for (size_t model_i = 0; model_i < cloud_models_with_normals_.size (); ++model_i)
	{
		cloud_models_with_normals.push_back (cloud_models_with_normals_[model_i]);

		PointCloud<PPFSignature>::Ptr cloud_model_ppf (new PointCloud<PPFSignature> ());
		PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;

		ppf_estimator.setInputCloud (cloud_models_with_normals_[model_i]);
		ppf_estimator.setInputNormals (cloud_models_with_normals_[model_i]);
		ppf_estimator.compute (*cloud_model_ppf);

		PPFHashMapSearch::Ptr hashmap_search (new PPFHashMapSearch (rot_thresh, tran_thresh));
		hashmap_search->setInputFeatureCloud (cloud_model_ppf);
		hashmap_search_vector.push_back (hashmap_search);
	}
	PCL_INFO ("Done\n");
	return 0;
}	

Eigen::Affine3d PatternPoseEstimation::detect(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
	if(cloud_with_normals->points.empty())
	{
		throw std::runtime_error("Empty cloud");
	}
	
	std::vector<Eigen::Affine3d> transforms_;

	try
	{		
		for (size_t model_i = 0; model_i < hashmap_search_vector.size (); ++model_i)
		{
			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
			PPFRegistration<PointNormal, PointNormal> ppf_registration;
			ppf_registration.setSceneReferencePointSamplingRate (1);
			ppf_registration.setPositionClusteringThreshold(cluster_tran_thresh);
			ppf_registration.setRotationClusteringThreshold(cluster_rot_thresh);
			ppf_registration.setSearchMethod(hashmap_search_vector[model_i]);
			ppf_registration.setInputSource(cloud_models_with_normals[model_i]);
			ppf_registration.setInputTarget(cloud_with_normals);
			
			Eigen::Matrix4f mat = ppf_registration.getFinalTransformation ();
			ppf_registration.align(*cloud_output_subsampled);
			mat = ppf_registration.getFinalTransformation ();
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

			begin = std::chrono::steady_clock::now();
			Eigen::Matrix4f refined_transform=refine(cloud_output_subsampled,cloud_with_normals);
			end = std::chrono::steady_clock::now();
			mat=refined_transform*mat;

			transforms_.push_back(Eigen::Affine3d (mat.cast<double>()));
		}
	}
	catch(std::exception &e)
	{
		throw std::exception(e);
	}

	return transforms_[0];
}

Eigen::Matrix4f PatternPoseEstimation::refine(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target)
{
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_target);
	pcl::PointCloud<pcl::PointNormal> cloud_output;
	icp.align(cloud_output);
	*cloud_output_subsampled=cloud_output;

	if(icp.getFitnessScore()>fitting_score_thresh)
	{
		std::cout << "icp did not converge" << std::endl;
		throw std::runtime_error("icp did not converge");
	}
	else
	{
		std::cout << "icp converged: "<< icp.getFitnessScore() << std::endl;		
	}
	
	return icp.getFinalTransformation();
}

