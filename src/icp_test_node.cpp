#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

class ICPProcess
{
  protected:
    ros::Publisher  pub_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //pcl::IterativeClosestPoint<VPoint, VPoint> icp;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in  ;//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out ;//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud;//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud_in;//(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud_in  ;//(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud_out ;//(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PolygonMesh::Ptr mesh;
    pcl::OrganizedFastMesh<pcl::PointXYZ> fast_mesh;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    //boost::shared_ptr< std::vector<Eigen::Vector3d> > covs;
    //std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >  covs_out;
    //std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >  covs_in;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::MatricesVectorPtr covs_out;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::MatricesVectorPtr covs_in;

    bool first_time;
    double uni_sampling_factor_;

    Eigen::Matrix4d transformation_matrix;
    Eigen::Matrix3d rot_matrix;
    Eigen::Vector3d trans_vector;

    tf::TransformBroadcaster tf_br_;
    tf::Transform curr_transform;

  public:
    ICPProcess(ros::NodeHandle & nh, double & uni_sampling_factor)
    {
      pub_ = nh.advertise<sensor_msgs::PointCloud2>( "filtered_points", 1);
      first_time = true;
      uni_sampling_factor_ = uni_sampling_factor;

      cloud_in  = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      orig_cloud= pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      orig_cloud_in= pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

      normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
      mesh = pcl::PolygonMesh::Ptr (new pcl::PolygonMesh);

      covs_in =     pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::MatricesVectorPtr (new     pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::MatricesVector) ;
      covs_out =     pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::MatricesVectorPtr (new     pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::MatricesVector) ;


      //icp.setMaximumIterations(50);
      // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
      icp.setMaxCorrespondenceDistance (0.05);
      // // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon (1e-8);
      // // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon (1);
      //pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ, pcl::PointXYZ,float>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ, pcl::PointXYZ, float>);
      //icp.setTransformationEstimation (trans_lls);

      curr_transform.setIdentity();

      gicp.setMaxCorrespondenceDistance(1);
    }
    ~ICPProcess()
    {
      //delete cloud_out;
      //delete cloud_in;

    }

    void pcCallback (const sensor_msgs::PointCloud2ConstPtr &scanMsg)
    {

      ros::Time time_begin = ros::Time::now();

      //VPointCloud::Ptr orig_cloud(new VPointCloud());
      //pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::fromROSMsg(*scanMsg , *orig_cloud);
      pcl::fromROSMsg(*scanMsg , *cloud_out);
      //outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      //outMsg->header.frame_id = "base_imu";
      cloud_out->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      cloud_out->header.frame_id = "base_imu";
      orig_cloud->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      orig_cloud->header.frame_id = "base_imu";



      pcl::PointCloud<int> indices_out;
      pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
      uniform_sampling.setInputCloud (orig_cloud);
      //uniform_sampling.setRadiusSearch (0.150f);
      uniform_sampling.setRadiusSearch (uni_sampling_factor_);
      //uniform_sampling.filter (*model_keypoints);
      //uniform_sampling.compute (indices_out);

      //*cloud_out = * orig_cloud;
      uniform_sampling.filter (*cloud_out);
      std::cout << "Model total points: " << orig_cloud->size () << "; Selected Keypoints: " << cloud_out->size () << std::endl;

      //std::cout << "Model total points: " << orig_cloud->size () << "; Selected Keypoints: " << indices_out.size () << std::endl;
      //pcl::copyPointCloud(*orig_cloud, indices_out.points, *cloud_out);

      //pcl::PointCloud<int> indices_out;
      //uniform_sampling.setInputCloud (cloud_in);
      //uniform_sampling.compute (*indices_out);
      //pcl::copyPointCloud(*orig_cloud, indices_out.points, *cloud_out);

      //fast_mesh.setInputCloud(cloud_out);
      //fast_mesh.reconstruct( * mesh);

      //pcl::features::computeApproximateNormals( *cloud_out, mesh->polygons, *normals);
      //pcl::features::computeApproximateCovariances( * cloud_out, * normals, *covs_out);

      if (!first_time)
      {

	gicp.setInputSource(cloud_in);
	gicp.setInputTarget(cloud_out);
	//gicp.setSourceCovariances(covs_in);
	//gicp.setTargetCovariances(covs_out);

	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
	//icp.setInputCloud(cloud_in);
	//icp.setInputTarget(cloud_out);
	//icp.align(*Final);
	gicp.align(*Final);

	std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
	  gicp.getFitnessScore() << std::endl;
	//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	  //icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;
	//transformation_matrix = icp.getFinalTransformation ().cast<double>();
	transformation_matrix = gicp.getFinalTransformation ().cast<double>();
	//trans_vector << transformation_matrix(0,3),transformation_matrix(1,3),transformation_matrix(2,3);
	tf::Matrix3x3 tf3d;
	tf3d.setValue(transformation_matrix(0,0),transformation_matrix(0,1),transformation_matrix(0,2)
			   ,transformation_matrix(1,0),transformation_matrix(1,1),transformation_matrix(1,2)
			   ,transformation_matrix(2,0),transformation_matrix(2,1),transformation_matrix(2,2)
	    );

	 tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
	tf::Transform transform_;
	transform_.setOrigin(tf::Vector3(transformation_matrix(0,3),transformation_matrix(1,3),transformation_matrix(2,3)));
	transform_.setRotation(tfqt);
	tf::StampedTransform transform;
	//transform.setData(transform_);
	transform.stamp_ = ros::Time::now();
	transform.frame_id_ = std::string("world");//ros::Time::now();
	transform.child_frame_id_ = std::string("velodyne");//ros::Time::now();
	//tf_br_.sendTransform(transform);
	curr_transform *= transform_;
	transform.setData(curr_transform);
	tf_br_.sendTransform(transform);


	//pub_.publish(Final);
      }

      //pub_.publish(outMsg);
      //pub_.publish(cloud_out);
      *cloud_in = *cloud_out;
      //cloud_in = cloud_out;
      *covs_in = * covs_out;
      *orig_cloud_in = *orig_cloud;

      first_time = false;

      ros::Duration dur = ros::Time::now() - time_begin;
      double ddur = dur.toNSec() * 1e-9;
      std::cout << "Elapsed:" << ddur << std::endl;

      // Obtain the transformation that aligned cloud_source to cloud_source_registered
       //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
       Eigen::Matrix4f transformation = gicp.getFinalTransformation ();


      pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> tve;
      tve.setMaxRange (0.01);  // 1cm
      tve.setThreshold (0.01);
      //double score = tve.validateTransformation (cloud_in, cloud_out, transformation);
      double score = tve.validateTransformation (orig_cloud_in, orig_cloud, transformation);
      std::cout << "Verify:" << score << " " << tve.isValid(cloud_in, cloud_out, transformation) << std::endl;

    }



};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp_test_node");
  ros::NodeHandle nh;//("~");
  //ros::NodeHandle nh("~");
  ros::Rate r(10);

  double uni_sampling_factor;
  //nh.getParam("uniform_sampling", uni_sampling_factor);
  nh.param("/icp_test_node/uniform_sampling", uni_sampling_factor, 0.15);

  ICPProcess icp_process(nh, uni_sampling_factor);

  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , &ICPProcess::pcCallback, &icp_process);

  while (ros::ok()){ros::spinOnce();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}

  return (0);
}
