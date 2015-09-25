/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  ros_pcl_icp.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */


#ifndef ROS_PCL_ICP_H_
#define ROS_PCL_ICP_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map_odom_icp/IcpSrv.h>

class RosPclIcp{

  public:
    RosPclIcp(ros::NodeHandle &nh);
    
    bool registerClouds(
      const sensor_msgs::PointCloud2 &target,
      const geometry_msgs::PoseStamped &target_pose,
      const sensor_msgs::PointCloud2 &cloud,
      const geometry_msgs::PoseStamped &cloud_pose,
      geometry_msgs::PoseStamped &result_pose,
      geometry_msgs::Transform &delta_transform,
      double correspondence_distance = 0.1,
      double transformation_epsilon = 1e-8,
      int maximum_iterations = 100,
      bool downsample_target = false,
      bool downsample_cloud = false,
      float downsample_leafsize = 0.02f);

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer service;

    bool registerCloudsSrv( map_odom_icp::IcpSrv::Request &req,
                            map_odom_icp::IcpSrv::Response &res);
  

    void tfToEigen( const tf::Transform &transform_tf,
                    Eigen::Matrix4f &transform_eigen);
    
    void eigenToTf( const Eigen::Matrix4f &transform_eigen,
                    tf::Transform &transform_tf);

    void downsampleCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
      float leafSize);

};

#endif /* ros_pcl_icp.h */
