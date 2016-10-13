/*
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include "std_msgs/String.h"
#include "semantic_mapper/Read.hpp"

//PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

namespace point_cloud_io {
// MOVE THIS
std::map<int, int> removeIdxMap;

Read::Read(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloudMessage_(new sensor_msgs::PointCloud2)
{
  if (!readParameters()) 
    ros::requestShutdown();
  // PointCloud2 publisher to publish semantic point cloud
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
  // PointCloud2 publisher to publish robot's point cloud
  robotMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/semantic_robot_map", 1, true);
  // PointCloud2 subscriber for new depth sensor scans
  pointCloudSubscriber_ = nodeHandle_.subscribe("/camera/depth_registered/points", 1, &Read::pointCloudCallback, this);
  // publish semantically labeled image
  //imagePublisher_ = imageTransport_.advertise("/semantic_img",1); 
  imagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/semantic_img", 1, true);
  // publish semantic map
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
    // example: http://www.pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

    // initialize the robot's updated semantic map as a colorless pointcloud 
    pcl::copyPointCloud(plyPointCloudXYZ_, robotMapCloud_);
    robotMapCloud_ptr_ = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >(robotMapCloud_);
    
 
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
  // save pointcloud as image
/*  std::stringstream fn;
  fn.precision(4);
  fn << "/home/brigit/jackrabbot/hallway_office/imgs/original/img_" << std::setfill('0') << std::setw(4) << count << ".png";
  pcl::io::savePNGFile(fn.str(), *temp_cloud_ptr);
  ROS_INFO_STREAM("saving file: " << fn.str());
  fn.str("");
  fn.clear();
*/
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
  //static const float radius(1); // 3 meters (office door->window distance) 

  // (0) Box search
  pcl::PointXYZ minPt;
  pcl::PointXYZ maxPt;
  std::vector<int> indices;
  pcl::getMinMax3D(*temp_cloud_xyz_ptr, minPt, maxPt);
  ROS_INFO_STREAM("min: (" << minPt.x << "," << minPt.y << ","  << minPt.z << ")");
  ROS_INFO_STREAM("max: (" << maxPt.x << "," << maxPt.y << ","  << maxPt.z << ")");

/*
  std::vector<double> d{(maxPt.x-minPt.x), (maxPt.y-minPt.y), (maxPt.z-minPt.z)};  
  std::vector<double> dimns(d);  
  std::sort(dimns.begin(), dimns.end());
  double radius = dimns[1]/2; 
  bool xIsMax = (d[0] > d[1] && d[0] > d[2]);
  bool yIsMax = (d[1] > d[0] && d[1] > d[2]);
  bool zIsMax = (d[2] > d[0] && d[2] > d[1]);
  std::vector<pcl::PointXYZ> kdTree_centers;

  pcl::PointXYZ startPt_tmp(minPt.x + radius, minPt.y + (maxPt.y-minPt.y)/2.0, minPt.z + (maxPt.z-minPt.z)/2.0);  

  if(xIsMax) {
    ROS_INFO_STREAM("x is max");
    // determine number of local kdtrees to create.
    float numCenters = ceil(d[0]/(2*radius));
    pcl::PointXYZ startPt(minPt.x + (maxPt.x-minPt.x)/2.0, minPt.y, minPt.z);  
    for(int i=0; i<numCenters; ++i) {
	kdTree_centers.push_back(startPt);
	startPt.x += 2*radius;
    }    
  } 
  if(yIsMax) {
    ROS_INFO_STREAM("y is max"); 
  } 
  if(zIsMax) {
    ROS_INFO_STREAM("z is max"); 
  } 

  ROS_INFO_STREAM("dims: (" << (maxPt.x-minPt.x) << ", " << (maxPt.y-minPt.y) << ", " << (maxPt.z-minPt.z)); 
*/

  Eigen::Vector4f minPtE(minPt.x, minPt.y, minPt.z, 0);
  Eigen::Vector4f maxPtE(maxPt.x, maxPt.y, maxPt.z, 0);
  pcl::getPointsInBox(plyPointCloudXYZ_, minPtE, maxPtE, indices); 
  ROS_INFO_STREAM("--------------------- # points found in box = " << indices.size());


  //kdTree for filtered points in min/max bounding box
  //create point cloud with filtered indices
//  boost::shared_ptr< const std::vector<int> > ind_ptr = boost::make_shared< const std::vector<int> >(indices); 
//  pcl::KdTreeFLANN<pcl::PointXYZ> radius_kdTree;
//  radius_kdTree.setInputCloud(plyPointCloudXYZ_ptr_, ind_ptr);  // indices into original semantic point cloud
  // kdtree of robot scan
  pcl::KdTreeFLANN<pcl::PointXYZ> robot_radius_kdTree;
  robot_radius_kdTree.setInputCloud(temp_cloud_xyz_ptr);
  
  // (0a) Create a kdTree with current robot scan & iterate over each point in the bounding box to find NN.
  BOOST_REVERSE_FOREACH (int idx, indices) { // iterates over indices in robot map backwards

    // compare a point in min/max bounding box to kdTree found in robot scan (newest pointcloud)
    pcl::PointXYZ xyzPt(robotMapCloud_.points[idx].x, robotMapCloud_.points[idx].y,robotMapCloud_.points[idx].z);
    // pointIdxRadiusSearch indices index into robot scan point clouds
    if ( robot_radius_kdTree.radiusSearch(xyzPt, 0.25, pointIdxRadiusSearch, pointRadiusSquaredDistance,1) <= 0 ) {
       // mark for deletion or update "seen" count
       if( removeIdxMap.find(idx) != removeIdxMap.end() ) {
          if( removeIdxMap[idx] > 8 ) {
             robotMapCloud_.points[idx].r = 32; //100; 
             robotMapCloud_.points[idx].g = 32; // 0;
             robotMapCloud_.points[idx].b = 32; //64;
           }
           else {
	     removeIdxMap[idx]++;
	   }
          
          //robotMapCloud_.points.erase(robotMapCloud_.points.begin() + idx);
          //ROS_INFO_STREAM("removing point idx: " << idx);
        }
	// if not in removeIdxMap, initialize
	else {	
	     removeIdxMap[idx] = 1;
        }
       
    }
    else {
       //temp_cloud_ptr->points[ pointIdxRadiusSearch[0] ].r = plyPointCloud_.points[idx].r; 
       //temp_cloud_ptr->points[ pointIdxRadiusSearch[0] ].g = plyPointCloud_.points[idx].g; 
       //temp_cloud_ptr->points[ pointIdxRadiusSearch[0] ].b = plyPointCloud_.points[idx].b; 
       robotMapCloud_.points[idx].r = plyPointCloud_.points[idx].r; 
       robotMapCloud_.points[idx].g = plyPointCloud_.points[idx].g;
       robotMapCloud_.points[idx].b = plyPointCloud_.points[idx].b;
       removeIdxMap[idx] = -1; // don't remove
    }
  }


  ROS_INFO_STREAM("finished updating robot map");
  sensor_msgs::PointCloud2 cloudOutRobotMap;
  ROS_INFO_STREAM("cloud size/h/w: " <<  robotMapCloud_.points.size() << " " << robotMapCloud_.height << " " << robotMapCloud_.width );
  robotMapCloud_.height = 1;
  robotMapCloud_.width = robotMapCloud_.points.size();
  pcl::toROSMsg(robotMapCloud_, cloudOutRobotMap);
  cloudOutRobotMap.header.frame_id = target;
  ROS_INFO_STREAM("Publishing point cloud!");
  robotMapPublisher_.publish(cloudOutRobotMap);

  sensor_msgs::PointCloud2 cloudOutLabel;
  pcl::toROSMsg(*temp_cloud_ptr, cloudOutLabel);
  cloudOutLabel.header.frame_id = target;
  // semantic labels image
  sensor_msgs::Image imgLabel;
  pcl::toROSMsg(cloudOutLabel, imgLabel);
  imagePublisher_.publish(imgLabel);

return;

  // (1) narrow search semantic space to points within a radius of scan point cloud centroid 
/*  Eigen::Vector4f centroid; 
  //pcl::compute3DCentroid(*temp_cloud_xyz_ptr, centroid); 
  pcl::compute3DCentroid(*temp_cloud_ptr, centroid); 
  pcl::PointXYZ centroidPt(centroid.x(),centroid.y(), centroid.z());
  //pcl::PointXYZRGB centroidPt(centroid.x(),centroid.y(), centroid.z());
  /////kdTree_.radiusSearch(centroidPt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  kdTree_.radiusSearch(startPt_tmp, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  // end (1)
  ROS_INFO_STREAM("centroid: (" << centroid.x() << "," << centroid.y() << ","  << centroid.z() << ")");
  ROS_INFO_STREAM("point reduction = " << pointIdxRadiusSearch.size() << "/" << (pointCloudMessage_->height*pointCloudMessage_->width));
  ROS_INFO_STREAM("percent reduction = " << 100*((double)(pointIdxRadiusSearch.size())/(pointCloudMessage_->height*pointCloudMessage_->width)) << "%");
*/

  // DEBUG: visualize filtered pointcloud
  //pcl::PointCloud<pcl::PointXYZ> radius_cloud(*plyPointCloudXYZ_ptr_, pointIdxRadiusSearch);
  //pcl::PointCloud<pcl::PointXYZRGB> radius_cloud(*plyPointCloud_ptr_, pointIdxRadiusSearch);
/*  pcl::PointCloud<pcl::PointXYZ> radius_cloud(*plyPointCloudXYZ_ptr_, indices); // min/max bounding box
  sensor_msgs::PointCloud2 radiusCloudOut;
  // Define PointCloud2 message and copy to ROS sensor_msg for publishing
  pcl::toROSMsg(radius_cloud, radiusCloudOut);
  radiusCloudOut.header.frame_id = target;
  ROS_INFO_STREAM("Publishing point cloud!");
  //transPointCloudPublisher_.publish(cloudOut);
  // DEBUG: 
  transPointCloudPublisher_.publish(radiusCloudOut);
*/
  // --------------------   

  // form new kdTree from filtered points (of original semantic point cloud)
  // convert filtered indices to boost shared pointer to create new kdTree
//  boost::shared_ptr< const std::vector<int> > ind_ptr = boost::make_shared< const std::vector<int> >(pointIdxRadiusSearch); 
  //boost::shared_ptr< const std::vector<int> > ind_ptr = boost::make_shared< const std::vector<int> >(indices); 

  //kdTree for filtered points
//  pcl::KdTreeFLANN<pcl::PointXYZ> radius_kdTree;
//  radius_kdTree.setInputCloud(plyPointCloudXYZ_ptr_, ind_ptr);  // indices into original semantic point cloud

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
  static const float nnRadius(0.13);
  // reset indices/distances
  pointIdxRadiusSearch.clear(); // these are the indices into the original point cloud right?
  pointRadiusSquaredDistance.clear();
  
  // to set rgb values: http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html
  //BOOST_FOREACH (pcl::PointXYZ& pt, temp_cloud_xyz_ptr->points) { 
  BOOST_FOREACH (pcl::PointXYZRGB& pt, temp_cloud_ptr->points) { 
    if( std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ) { 
	pt.x = 999; pt.y = 999; pt.z = 999;
    }

    pcl::PointXYZ xyzPt(pt.x, pt.y, pt.z);
    // nearest neighbor search which is very slow 
    //int nn(0);
   /* if((nn = radius_kdTree.nearestKSearch(pt, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance)) > 0 )
    {
       ROS_INFO_STREAM("Found " << nn << " nearest neighbors."); 
    }
   */

    // pointIdxRadiusSearch indices index into original point clouds
///    if ( radius_kdTree.radiusSearch(xyzPt, nnRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance,1) > 0 ) { 

     //ROS_INFO_STREAM("Index " << pointIdxRadiusSearch[0] );
     //ROS_INFO_STREAM("semantic pcl size: " <<  plyPointCloud_.points.size() );
///     pt.r = plyPointCloud_.points[ pointIdxRadiusSearch[0] ].r; 
///     pt.g = plyPointCloud_.points[ pointIdxRadiusSearch[0] ].g; 
///     pt.b = plyPointCloud_.points[ pointIdxRadiusSearch[0] ].b; 
///     robotMapCloud_.points[ pointIdxRadiusSearch[0] ] = pt;

     //temp_cloud_proj_ptr->points[ pointIdxRadiusSearch[0] ].z = 0;
     //ROS_INFO_STREAM("Found " << pointIdxRadiusSearch.size() << " neighbors." );
     //ROS_INFO_STREAM("Index " << pointIdxRadiusSearch[0] );

     /*for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
      std::cout << "    "  <<   temp_cloud_xyz_ptr->points[ pointIdxRadiusSearch[i] ].x 
                << " " << temp_cloud_xyz_ptr->points[ pointIdxRadiusSearch[i] ].y 
                << " " << temp_cloud_xyz_ptr->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
 
      }
    */
///    }

    // for projected map
    //pt.z = 0;
   
 }

  //ex rgbd->ros: https://github.com/tue-robotics/rgbd/blob/master/src/rgbd_to_ros.cpp
  // semantic labels for point cloud
/*  sensor_msgs::PointCloud2 cloudOutLabel;
  pcl::toROSMsg(*temp_cloud_ptr, cloudOutLabel);
  cloudOutLabel.header.frame_id = target;
  // semantic labels image
  sensor_msgs::Image imgLabel;
  pcl::toROSMsg(cloudOutLabel, imgLabel);
*/
/*  fn << "/home/brigit/jackrabbot/hallway_office/imgs/semantic/img_semantic_" << std::setfill('0') << std::setw(4) << count << ".png";
  pcl::io::savePNGFile(fn.str(), *temp_cloud_ptr);
  ROS_INFO_STREAM("saving file: " << fn.str());
*/  
  // robot updated semantic map
/*  sensor_msgs::PointCloud2 cloudOutRobotMap;
  pcl::toROSMsg(robotMapCloud_, cloudOutRobotMap);
  cloudOutRobotMap.header.frame_id = target;

  // publish transformed pointcloud 
  ROS_INFO_STREAM("Publishing point cloud!");
  //transPointCloudPublisher_.publish(cloudOut);
  // DEBUG: 
  //transPointCloudPublisher_.publish(radiusCloudOut);
  //////transPointCloudPublisher_.publish(cloudOutLabel);
  robotMapPublisher_.publish(cloudOutRobotMap);
  imagePublisher_.publish(imgLabel);
*/
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
