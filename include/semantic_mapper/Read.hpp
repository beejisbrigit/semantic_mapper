/*
 * Read.hpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// tf
#include <tf/transform_listener.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>


namespace point_cloud_io {

class Read
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Read(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Read();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initializes node.
   */
  void initialize();

  /*!
   * Read the point cloud from a .ply or .vtk file.
   * @param filePath the path to the .ply or .vtk file.
   * @param pointCloudFrameId the id of the frame of the point cloud data.
   * @return true if successful.
   */
  bool readFile(const std::string& filePath, const std::string& pointCloudFrameId);

  /*!
   * PointCloud callback function.
   * @param timerEvent the timer event.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  /*!
   * Timer callback function.
   * @param timerEvent the timer event.
   */
  void timerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Publish the point cloud as a PointCloud2.
   * @return true if successful.
   */
  bool publish();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud message to publish.
  sensor_msgs::PointCloud2::Ptr pointCloudMessage_;

  //! Point cloud subscriber for depth sensor
  ros::Subscriber pointCloudSubscriber_;

  //! Point cloud publisher.
  ros::Publisher transPointCloudPublisher_;

  //! Point cloud publisher.
  ros::Publisher pointCloudPublisher_;

  //! Timer for publishing the point cloud.
  ros::Timer timer_;

  //! Path to the point cloud file.
  std::string filePath_;

  //! Point cloud topic to be published at.
  std::string pointCloudTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  //! If true, continous publishing is used.
  //! If false, point cloud is only published once.
  bool isContinousPublishing_;

  //! Duration between publishing steps.
  ros::Duration updateDuration_;

  //! Transform listener for source->target transform
  tf::TransformListener tf_listener_;

  //! Point cloud loaded from PLY file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plyPointCloud_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB> plyPointCloud_;

  //! Point cloud loaded from PLY file
  pcl::PointCloud<pcl::PointXYZ>::Ptr plyPointCloudXYZ_ptr_;
  pcl::PointCloud<pcl::PointXYZ> plyPointCloudXYZ_;

  //! KdTree for finding NNs
  pcl::KdTreeFLANN<pcl::PointXYZ> kdTree_;
  // pcl::KdTreeFLANN<pcl::PointXYZRGB> kdTree_;
  //pcl::search::KdTree<pcl::PointXYZRGB> kdTree_;
};

} /* namespace */
