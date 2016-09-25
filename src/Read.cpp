/*
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <string>
#include <cmath>
#include "std_msgs/String.h"
#include "semantic_mapper/Read.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

namespace point_cloud_io {

Read::Read(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloudMessage_(new sensor_msgs::PointCloud2)
{
  if (!readParameters()) 
    ros::requestShutdown();
  // PointCloud2 publisher to publish semantic point cloud
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
  // PointCloud2 subscriber for new depth sensor scans
  pointCloudSubscriber_ = nodeHandle_.subscribe("/camera/depth_registered/points", 1, &Read::pointCloudCallback, this);
  //pointCloudSubscriber_ = nodeHandle_.subscribe("/camera/depth/points", 1, &Read::pointCloudCallback, this);
  transPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/trans_depth_points", 1, true);
  // publish from here
  initialize();

  //}


}

Read::~Read()
{

}

bool Read::readParameters()
{
  bool allParametersRead = true;
  if (!nodeHandle_.getParam("file_path", filePath_)) allParametersRead = false;
  if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;
  if (!nodeHandle_.getParam("frame", pointCloudFrameId_)) allParametersRead = false;

  double updateRate;
  nodeHandle_.param("rate", updateRate, 0.0);
  if (updateRate == 0.0)
  {
    isContinousPublishing_ = false;
  }
  else
  {
    isContinousPublishing_ = true;
    updateDuration_.fromSec(1.0 / updateRate);
  }

  if (!allParametersRead)
  {
    ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io read"
        " _file_path:=/home/user/my_point_cloud.ply"
        " _topic:=/my_topic"
        " _frame:=sensor_frame"
        " (optional: _rate:=publishing_rate)");
    return false;
  }

  return true;
}

void Read::initialize()
{
  // read PLY file with pointcloud
  if (!readFile(filePath_, pointCloudFrameId_)) ros::requestShutdown();

  // create a ros::Timer to trigger timerCallback where pointcloud publishing occurs every
  // updateDuration_ (can also be 0 for single publish?)
  if (isContinousPublishing_)
  {
    // creating timer triggers timer callback from which point cloud is published  
    timer_ = nodeHandle_.createTimer(updateDuration_, &Read::timerCallback, this);
  }
  // single publish - no callback
  else
  {
    Duration(1.0).sleep(); // Need this to get things ready before publishing.
    if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
    ros::requestShutdown();
  }
}

bool Read::readFile(const std::string& filePath, const std::string& pointCloudFrameId)
{
  if (filePath.find(".ply") != std::string::npos) {
    // Load .ply file.
    if (loadPLYFile(filePath, plyPointCloud_) != 0) 
       return false;

    // Define PointCloud2 message and copy to ROS sensor_msg for publishing
    pcl::toROSMsg(plyPointCloud_, *pointCloudMessage_);

    // XYZ 
    //pcl::PointCloud<PointXYZ>::Ptr plyPointCloud_xyz_ptr(new pcl::PointCloud<PointXYZ>);
    //pcl::copyPointCloud(plyPointCloud_, *plyPointCloud_xyz_ptr);
    //kdTree_.setInputCloud(plyPointCloud_xyz_ptr); 
    pcl::copyPointCloud(plyPointCloud_, plyPointCloudXYZ_);
    plyPointCloudXYZ_ptr_ = boost::make_shared< pcl::PointCloud<pcl::PointXYZ> >(plyPointCloudXYZ_);
    kdTree_.setInputCloud(plyPointCloudXYZ_ptr_); 

    // XYZRGB doesn't seem to work: create kdTree for semantic point cloud
    //plyPointCloud_ptr_ = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >(plyPointCloud_);
    //kdTree_.setInputCloud(plyPointCloud_ptr_); 
    // example: http://www.pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search
 
  }
  else {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  ROS_INFO_STREAM("Loaded point cloud with " << pointCloudMessage_->height*pointCloudMessage_->width << " points.");
  return true;
}

void Read::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  clock_t start = clock();
  static int count = 0;
  ROS_INFO_STREAM("Received new robot scan (point cloud) with " << cloud->height*cloud->width << " points.");

  // transform received point cloud into /map (world) coordinates
  tf::StampedTransform transform;
  // for Primesense, source frame should be /camera_depth_optical_frame
  static const std::string source("/camera_depth_optical_frame"); // originating data frame 
  static const std::string target("/map"); // frame to which data should be transformed
 
  try {
    // this call will block until transform is received.
    tf_listener_.waitForTransform(target, source,  
                               ros::Time(0), ros::Duration(0.1)); // block for 0.1 secs
    tf_listener_.lookupTransform(target, source,  
                               ros::Time(0), transform);
    ROS_INFO_STREAM("Got transform " << source << "->" << target << ": [ x: "
	 << transform.getOrigin().x() << " y: "
	 << transform.getOrigin().y() << " z: " 
	 << transform.getOrigin().z() << " ]");
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  
  // example: http://answers.ros.org/question/90246/pcl-to-tf-example-for-pcltransformpointcloud/
  // API: http://docs.ros.org/api/pcl_ros/html/namespacepcl__ros.html
  // transform point cloud into target (/map) reference frame.
  sensor_msgs::PointCloud2 cloudOut;
  pcl_ros::transformPointCloud(target, transform, *cloud, cloudOut);

  // convert from sensor_messages::PointCloud2->pcl::PCLPointCloud2->pcl native pcl::PointXYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud_ptr); // dereference pointer to pointer
  pcl::fromROSMsg(cloudOut, *temp_cloud_ptr); // dereference pointer to pointer
  ROS_INFO("ros->pcl_xzyrgb");

  // convert current point cloud to pcl::PointXYZ for kdtree NN search
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>); // need to use ctr!!
  pcl::copyPointCloud(*temp_cloud_ptr, *temp_cloud_xyz_ptr); // copy PointXYZRGBs to PointXYZs
  ROS_INFO("pcl_xzyrgb->pcl_xyz");

  // DEBUG: test transformed point clouds 
  //sensor_msgs::PointCloud2 cloudOutTest;
  //pcl::toROSMsg(*temp_cloud_xyz_ptr, cloudOutTest);
  //cloudOutTest.header.frame_id = target;

  // kdTree NN search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  static const float radius(1); // 3 meters (office door->window distance) 

  // narrow search semantic space to points within a radius of scan point cloud centroid 
  Eigen::Vector4f centroid; 
  //pcl::compute3DCentroid(*temp_cloud_xyz_ptr, centroid); 
  pcl::compute3DCentroid(*temp_cloud_ptr, centroid); 
  pcl::PointXYZ centroidPt(centroid.x(),centroid.y(), centroid.z());
  //pcl::PointXYZRGB centroidPt(centroid.x(),centroid.y(), centroid.z());
  kdTree_.radiusSearch(centroidPt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  ROS_INFO_STREAM("centroid: (" << centroid.x() << "," << centroid.y() << ","  << centroid.z() << ")");
  ROS_INFO_STREAM("point reduction = " << pointIdxRadiusSearch.size() << "/" << (pointCloudMessage_->height*pointCloudMessage_->width));
  ROS_INFO_STREAM("percent reduction = " << 100*((double)(pointIdxRadiusSearch.size())/(pointCloudMessage_->height*pointCloudMessage_->width)) << "%");


  // DEBUG: visualize filtered pointcloud
  //pcl::PointCloud<pcl::PointXYZ> radius_cloud(*plyPointCloudXYZ_ptr_, pointIdxRadiusSearch);
  //pcl::PointCloud<pcl::PointXYZRGB> radius_cloud(*plyPointCloud_ptr_, pointIdxRadiusSearch);
  //sensor_msgs::PointCloud2 radiusCloudOut;
  // Define PointCloud2 message and copy to ROS sensor_msg for publishing
  //pcl::toROSMsg(radius_cloud, radiusCloudOut);
  //radiusCloudOut.header.frame_id = target;
  //ROS_INFO_STREAM("Publishing point cloud!");
  //transPointCloudPublisher_.publish(cloudOut);
  // DEBUG: 
  //transPointCloudPublisher_.publish(radiusCloudOut);

  // --------------------   

  // form new kdTree from filtered points (of original semantic point cloud)
  // convert filtered indices to boost shared pointer to create new kdTree
  boost::shared_ptr< const std::vector<int> > ind_ptr = boost::make_shared< const std::vector<int> >(pointIdxRadiusSearch); 

  //kdTree for filtered points
  pcl::KdTreeFLANN<pcl::PointXYZ> radius_kdTree;
  radius_kdTree.setInputCloud(plyPointCloudXYZ_ptr_, ind_ptr);
  //pcl::KdTreeFLANN<pcl::PointXYZRGB> radius_kdTree;
  //radius_kdTree.setInputCloud(plyPointCloud_ptr_, ind_ptr);

/* DEBUG: check indices  
  int cnt(0);
  std::vector<int> v = *(radius_kdTree.getIndices());
  // compare indices! ind_ptr != radius_kdTree indices probably
  BOOST_FOREACH(int i, *ind_ptr) {
	ROS_INFO_STREAM("kdd idx: " << i << " kdd radius idx: " << v[cnt++]);
  }
*/

  //static const int k(1);
  static const float nnRadius(0.15);
  //std::vector<float> minDistancePoints(640*480, 0.0);
  // reset indices/distances
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  // to set rgb values: http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html
  //BOOST_FOREACH (pcl::PointXYZ& pt, temp_cloud_xyz_ptr->points) { 
  BOOST_FOREACH (pcl::PointXYZRGB& pt, temp_cloud_ptr->points) { 
    if( std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ) { // look up why this happens!
	pt.x = 999; pt.y = 999; pt.z = 999;
    }

    //pt.r = 0; pt.g = 255; pt.b = 0;

    // uint8_t r = 0, g = 255, b = 0;
    //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    //pt.rgb = *reinterpret_cast<float*>(&rgb);

    //ROS_INFO_STREAM( "pt r/g/b: " << static_cast<int>(pt.r) << " " << static_cast<int>(pt.g) << " " << static_cast<int>(pt.b));
     //ROS_INFO_STREAM( "pts: " << pt.x << " " << pt.y << " "  << pt.z);
    // nearest neighbor search which is very slow 
//    int nn(0);
/*    if((nn = radius_kdTree.nearestKSearch(pt, k, pointIdxRadiusSearch, pointRadiusSquaredDistance)) > 0 )
    {
       //ROS_INFO_STREAM("Found " << nn << " nearest neighbors."); 
    }
*/
    pcl::PointXYZ xyzPt(pt.x, pt.y, pt.z);
    // pointIdxRadiusSearch indices index into original point clouds
    if ( radius_kdTree.radiusSearch(xyzPt, nnRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance,1) > 0 ) { 

      //pt.r = 0; pt.g = 255; pt.b = 0;
     //ROS_INFO_STREAM("Index " << pointIdxRadiusSearch[0] );
     //ROS_INFO_STREAM("semantic pcl size: " <<  plyPointCloud_.points.size() );
      pt.r = plyPointCloud_.points[ pointIdxRadiusSearch[0] ].r; 
      pt.g = plyPointCloud_.points[ pointIdxRadiusSearch[0] ].g; 
      pt.b = plyPointCloud_.points[ pointIdxRadiusSearch[0] ].b; 
     //ROS_INFO_STREAM("Found " << pointIdxRadiusSearch.size() << " neighbors." );
     //ROS_INFO_STREAM("Index " << pointIdxRadiusSearch[0] );

    /*for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
      std::cout << "    "  <<   temp_cloud_xyz_ptr->points[ pointIdxRadiusSearch[i] ].x 
                << " " << temp_cloud_xyz_ptr->points[ pointIdxRadiusSearch[i] ].y 
                << " " << temp_cloud_xyz_ptr->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
 
      }
    */
    }

    //ROS_INFO("------------------------------------");
    //    
   
 }

  sensor_msgs::PointCloud2 cloudOutLabel;
  pcl::toROSMsg(*temp_cloud_ptr, cloudOutLabel);
  cloudOutLabel.header.frame_id = target;

  // publish transformed pointcloud 
  ROS_INFO_STREAM("Publishing point cloud!");
  //transPointCloudPublisher_.publish(cloudOut);
  // DEBUG: 
  //transPointCloudPublisher_.publish(radiusCloudOut);
  transPointCloudPublisher_.publish(cloudOutLabel);

  clock_t ticks = clock() - start;
  ROS_INFO("Elapsed time: %f secs", (double)ticks/CLOCKS_PER_SEC );
  ROS_INFO_STREAM("---- count " << count++ << " ----"); 
}

void Read::timerCallback(const ros::TimerEvent& timerEvent)
{ 
  // publish message if ros::Timer is triggered (at specific rate in param file) 
  if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
}

bool Read::publish()
{
  // publish PointCloudMessage
  pointCloudMessage_->header.stamp = Time::now();
  if (pointCloudPublisher_.getNumSubscribers() > 0u)
  {
    pointCloudPublisher_.publish(pointCloudMessage_);
    ROS_INFO_STREAM("Point cloud published to topic \"" << pointCloudTopic_ << "\".");
  }
  return true;
}

} /* namespace */
