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
 *  map_odom_icp.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef MAP_ODOM_ICP_H_
#define MAP_ODOM_ICP_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros_pcl_icp.h>
#include <stack>
#include <boost/thread/mutex.hpp>
#include <map_odom_icp/MapOdomIcpConfig.h>
#include <dynamic_reconfigure/server.h>

class MapOdomICP{

  public:
    MapOdomICP(ros::NodeHandle &nh);
    void sendMapToOdomCombined();
  private:
    RosPclIcp icp_;

    class Sync{
      public:
        static void setUpdated();
        static bool hasUpdated();
        static void setTransform(tf::StampedTransform& tf);
        static void getTransform(tf::StampedTransform& tf);
      private:
        static boost::mutex mtx_; 
        static bool updated_;
        static tf::StampedTransform transform_;
    };

    boost::mutex target_mtx_;

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher cloud_pub_;
    dynamic_reconfigure::Server<map_odom_icp::MapOdomIcpConfig> dr_server;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    //std::stack<tf::StampedTransform>map_to_odom_;
    //std::stack<sensor_msgs::PointCloud2> clouds_;
    //std::stack<geometry_msgs::PoseStamped> poses_;

    sensor_msgs::PointCloud2::ConstPtr target_cloud_;
    sensor_msgs::PointCloud2::ConstPtr previous_cloud_;

    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped previous_pose_;
    tf::StampedTransform map_to_odom_;

    void storeTargetCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud);
    void pointCloud2Callback(const::sensor_msgs::PointCloud2::ConstPtr &cloud);
    
    bool icpWithTargetCloud(
      const sensor_msgs::PointCloud2::ConstPtr& cloud,
      const geometry_msgs::PoseStamped& cloud_pose
    );
    
    bool icpWithPreviousCloud(
      const sensor_msgs::PointCloud2::ConstPtr& cloud,
      const geometry_msgs::PoseStamped& cloud_pose
    );

    void updateMapOdomTransform(geometry_msgs::Transform& delta_transform);
    void resetMapOdomTransform();
    void reconfigureCallback(map_odom_icp::MapOdomIcpConfig& config, uint32_t level);

    bool icpUpdateMapToOdomCombined(
      const sensor_msgs::PointCloud2::ConstPtr& target,
      const geometry_msgs::PoseStamped& target_pose,
      const sensor_msgs::PointCloud2::ConstPtr& cloud,
      const geometry_msgs::PoseStamped& cloud_pose
    );
    
    bool getPointCloudPose(
      const sensor_msgs::PointCloud2 &cloud,
      const std::string &fixed_frame,
      geometry_msgs::PoseStamped &pose);

    // ros_pcl parameters
    double correspondence_distance_;
    double transformation_epsilon_;
    int maximum_iterations_;
    bool downsample_target_;
    bool downsample_cloud_;
    double downsample_leafsize_;
    
    bool publish_cloud_;
    bool use_target_cloud_;

};

#endif /* map_odom_icp.h */
