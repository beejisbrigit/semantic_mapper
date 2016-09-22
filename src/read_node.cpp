/*
 * read_node.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "semantic_mapper/Read.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_mapper");
  ros::NodeHandle nodeHandle("~");

  point_cloud_io::Read read(nodeHandle);

  ros::spin();
  return 0;
}
