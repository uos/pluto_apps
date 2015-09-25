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



#include <map_odom_icp.h>

MapOdomICP::MapOdomICP(ros::NodeHandle &nh)
 :  nh_(nh),
    icp_(nh)
{
  tf::Transform null_transform;
  null_transform.setIdentity();
  map_to_odom_ = tf::StampedTransform(null_transform, ros::Time::now(), "map", "odom_combined");
  cloud_sub_ = nh_.subscribe("input_cloud", 20, &MapOdomICP::pointCloud2Callback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);
  
  ros::NodeHandle private_nh("~");
  // ros pcl parameters
  private_nh.param("correspondence_distance", correspondence_distance_, 0.1);
  private_nh.param("transformation_epsilon",transformation_epsilon_, 1e-8);
  private_nh.param("maximum_iterations", maximum_iterations_, 20);
  private_nh.param("downsample_target", downsample_target_, false);
  private_nh.param("downsample_cloud", downsample_cloud_, false);
  private_nh.param("downsample_leafsize", downsample_leafsize_, 0.01);

  // Publish cloud after registration
  private_nh.param("publish_cloud", publish_cloud_, true);
  // do not do iterative icp with the previous cloud -> use topic target_cloud
  private_nh.param("use_target_cloud", use_target_cloud_, false);

  if(use_target_cloud_)
    target_sub_ = nh_.subscribe("target_cloud", 1, &MapOdomICP::storeTargetCloud, this);
}

bool MapOdomICP::getPointCloudPose( const sensor_msgs::PointCloud2 &cloud,
                                    const std::string &fixed_frame,
                                    geometry_msgs::PoseStamped &pose){

  std::string error_msg;
  bool success = tf_listener_.waitForTransform(fixed_frame, cloud.header.frame_id, cloud.header.stamp,
      ros::Duration(3.0), ros::Duration(0.01), &error_msg);
  if (!success)
  {
    ROS_WARN("Could not get transform! %s", error_msg.c_str());
    return false;
  }

  tf::StampedTransform transform;
  ROS_INFO("Lookup transform from %s to %s at %f sec",  
    fixed_frame.c_str(), cloud.header.frame_id.c_str(), cloud.header.stamp.toSec());
  try
  {
    tf_listener_.lookupTransform(fixed_frame, cloud.header.frame_id,  cloud.header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  tf::Stamped<tf::Transform> transform_pose(transform, transform.stamp_, transform.frame_id_);
  tf::poseStampedTFToMsg(transform_pose, pose);

  return true;
}

void MapOdomICP::storeTargetCloud(const sensor_msgs::PointCloud2::ConstPtr &target){
  target_mtx_.lock();
  ROS_INFO("received new target cloud.");
  target_cloud_ = target; 
  getPointCloudPose(*target, "map", target_pose_);
  target_mtx_.unlock();
}
void MapOdomICP::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  bool succeeded = true;
  bool first = true;
  if(use_target_cloud_){
    target_mtx_.lock();
    if(target_cloud_ != 0){
      succeeded = icpWithTargetCloud(cloud);    
    }else{
      ROS_INFO("No target cloud set");
    }
    target_mtx_.unlock();
  }
  else{
    if(previous_cloud_ != 0) {
      succeeded = icpWithPreviousCloud(cloud);   
    }else{
      first = getPointCloudPose(*cloud, "map", previous_pose_);
      ROS_INFO("Received first cloud.");
    }
  }

  if(first) {
    previous_cloud_ = cloud;
  }
  
  if(succeeded){
    Sync::setUpdated();
  }
}

bool MapOdomICP::icpWithTargetCloud(
  const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  geometry_msgs::PoseStamped result_pose;
  return icpUpdateMapToOdomCombined(
    target_cloud_,
    target_pose_,
    cloud,
    result_pose
  );
}

bool MapOdomICP::icpWithPreviousCloud(
  const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  return icpUpdateMapToOdomCombined(
    previous_cloud_,
    previous_pose_,
    cloud,
    previous_pose_
  );
}

bool MapOdomICP::icpUpdateMapToOdomCombined(
    const sensor_msgs::PointCloud2::ConstPtr &target,
    const geometry_msgs::PoseStamped &target_pose,
    const sensor_msgs::PointCloud2::ConstPtr &cloud,
    geometry_msgs::PoseStamped &result_pose
){
  bool succeeded = true;
  ROS_INFO("ros_icp received new point cloud");
  
  geometry_msgs::PoseStamped cloud_pose;
  getPointCloudPose(*cloud, "map", cloud_pose);
  

  geometry_msgs::PoseStamped tmp_result_pose;
  geometry_msgs::Transform delta_transform;

  // register point clouds with icp
  ROS_INFO("register point clouds...");
  succeeded = icp_.registerClouds(
    *target,
    target_pose,
    *cloud,
    cloud_pose,
    tmp_result_pose,
    delta_transform,
    correspondence_distance_,
    transformation_epsilon_,
    maximum_iterations_,
    downsample_target_,
    downsample_cloud_,
    downsample_leafsize_);

  if(succeeded){
    ROS_INFO("... register point clouds done.");     

    // convert to tf
    tf::Transform delta_transform_tf;
    tf::transformMsgToTF(delta_transform, delta_transform_tf);

    // calculate the new map to odom_combined transformation
    tf::Transform update_transform_tf = map_to_odom_ * delta_transform_tf;   

    tf::StampedTransform update_transform_stamped_tf(
      update_transform_tf, cloud->header.stamp, "map", "odom_combined");
    map_to_odom_ = update_transform_stamped_tf;
    
    // update the cloud pose
    result_pose.pose = tmp_result_pose.pose;
    result_pose.header = tmp_result_pose.header;
  }
  else{
    ROS_INFO("register point clouds failed.");
  }
  return succeeded;
}


bool MapOdomICP::Sync::updated_ = false;
boost::mutex MapOdomICP::Sync::mtx_;

void MapOdomICP::Sync::setUpdated(){
  mtx_.lock();
  updated_ = true;
  mtx_.unlock();
}

bool MapOdomICP::Sync::hasUpdated(){
  bool ret = false;
  mtx_.lock();
  if(updated_){
    updated_ = false;
    ret = true;
  }
  mtx_.unlock();
  return ret;
}

void MapOdomICP::sendMapToOdomCombined(){

  ros::Time now = ros::Time::now();

  tf_broadcaster_.sendTransform(
    tf::StampedTransform(
      map_to_odom_,
      now,
      "map",
      "odom_combined"
    )
  );


  if (Sync::hasUpdated()){
    tf::Transform transform = map_to_odom_;
    tf::Vector3 origin = transform.getOrigin();
    tf::Matrix3x3 basis = transform.getBasis();
    tfScalar roll, pitch, yaw;
    basis.getRPY(roll, pitch, yaw);

    ROS_INFO("Updated Transform from map to odom: The new transformation is \n (x:%f y:%f z:%f)[meter] (roll:%f pitch:%f yaw:%f)[degree]",
      origin.getX(),
      origin.getY(),
      origin.getY(),
      roll * 180 / M_PI,
      pitch * 180 / M_PI,
      yaw * 180 / M_PI);

    if(publish_cloud_){
      sensor_msgs::PointCloud2 cloud = *previous_cloud_;
      cloud.header.stamp = now;
      cloud_pub_.publish(cloud);
    }
  }
}

int main(int args, char** argv){
  ros::init(args, argv, "map_odom_icp");
  ros::NodeHandle nh;

  MapOdomICP map_odom_icp(nh);
  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ros::Rate rate(10.0);
  while(nh.ok()){
    map_odom_icp.sendMapToOdomCombined();
    rate.sleep();
  }
}
