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
#include <pcl_ros/transforms.h>


bool MapOdomICP::Sync::updated_ = false;
boost::mutex MapOdomICP::Sync::mtx_;
tf::StampedTransform MapOdomICP::Sync::transform_;

void MapOdomICP::Sync::setTransform(tf::StampedTransform& tf){
  mtx_.lock();
  transform_ = tf;
  updated_ = true;
  mtx_.unlock();
}

void MapOdomICP::Sync::getTransform(tf::StampedTransform& tf){
  mtx_.lock();
  tf = transform_;
  mtx_.unlock();
}

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

MapOdomICP::MapOdomICP(ros::NodeHandle &nh)
 :  nh_(nh),
    icp_(nh)
{
  resetMapOdomTransform();

  cloud_sub_ = nh_.subscribe("input_cloud", 1, &MapOdomICP::pointCloud2Callback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);

  dynamic_reconfigure::Server<map_odom_icp::MapOdomIcpConfig>::CallbackType dr_cb_type
    = boost::bind(&MapOdomICP::reconfigureCallback, this, _1, _2);
  dr_server.setCallback(dr_cb_type);

  ros::NodeHandle private_nh("~");
  // ros pcl parameters
  private_nh.param("max_corres_dist", correspondence_distance_, 0.1);
  private_nh.param("max_tf_epsilon",transformation_epsilon_, 1e-8);
  private_nh.param("max_iterations", maximum_iterations_, 20);
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

void MapOdomICP::reconfigureCallback(map_odom_icp::MapOdomIcpConfig& config, uint32_t level){
  
  ROS_INFO("resetting map to odom transformation.");
  resetMapOdomTransform();

/* Parameters:
  config.max_corres_dist;
  config.max_tf_epsilon;
  config.max_iterations;
  config.max_rejection_dist;
  config.max_rejection_angle;
  config.downsample_target;
  config.downsmaple_source;
  config.downsmaple_leafsize;
  config.icp_type;
*/

  ROS_INFO("\n Parameters changed to:\n Correspondence Distance: %f \n Transformation Epsilon: %e \n Downsample Target: %s \n Downsample Source: %s \n Downsample Leaf Size: %f \n Maximum Iterations: %d \n Maximum Rejection Distance: %f \n Maximum Rejection Angle: %f \n ICP Type: %d",
  config.max_corres_dist,
  config.max_tf_epsilon,
  config.downsample_target ? "true" : "false",
  config.downsample_source ? "true" : "false",
  config.downsample_leafsize,
  config.max_iterations,
  config.max_rejection_dist,
  config.max_rejection_angle,
  config.icp_type);
  
  target_mtx_.lock();
  icp_.setCorrespondenceDistance(config.max_corres_dist);
  icp_.setTransformationEpsilon(config.max_tf_epsilon);
  icp_.setDownsampleTarget(config.downsample_target);
  icp_.setDownsampleSource(config.downsample_source);
  icp_.setDownsampleLeafSize(config.downsample_leafsize);
  icp_.setMaximumIterations(config.max_iterations);
  icp_.setMaximumRejectionDistance(config.max_rejection_dist);
  icp_.setMaximumRejectionAngle(config.max_rejection_angle);
  icp_.setIcpType(config.icp_type);
  
  if(icp_.isTargetCloudSet()){
    ROS_INFO("preparing target cloud."); 
    icp_.prepareTargetCloud();
  }
  target_mtx_.unlock(); 
}

void MapOdomICP::resetMapOdomTransform(){
  tf::Transform null_transform;
  null_transform.setIdentity();
  tf::StampedTransform initial_transform(null_transform, ros::Time::now(), "map", "odom_combined");
  Sync::setTransform(initial_transform);
}

bool MapOdomICP::getOdomCombinedTransform( 
    const std::string& sensor_frame, 
    const ros::Time& time,
    tf::StampedTransform& transform)
{
  std::string error_msg;
  bool success = tf_listener_.waitForTransform("odom_combined", sensor_frame, time,
      ros::Duration(3.0), ros::Duration(0.01), &error_msg);
  if (!success)
  {
    ROS_WARN("Could not get transform! %s", error_msg.c_str());
    return false;
  }
  
  tf_listener_.lookupTransform ("odom_combined", sensor_frame, time, transform);
  return true;
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
  ROS_DEBUG("Lookup transform from %s to %s at %f sec",  
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
  ROS_INFO("Received new target cloud.");
  geometry_msgs::PoseStamped target_pose;
  if (getPointCloudPose(*target, "map", target_pose)){
    tf::Stamped<tf::Pose> transform;
    tf::poseStampedMsgToTF(target_pose, transform);
    
    tf::Vector3 origin = transform.getOrigin();
    tf::Matrix3x3 basis = transform.getBasis();
    tfScalar roll, pitch, yaw;
    basis.getRPY(roll, pitch, yaw);

    ROS_INFO("The Pose of the target point cloud is: \n (x:%f y:%f z:%f)[meter] (roll:%f pitch:%f yaw:%f)[degree]",
      origin.getX(),
      origin.getY(),
      origin.getY(),
      roll * 180 / M_PI,
      pitch * 180 / M_PI,
      yaw * 180 / M_PI);

    icp_.setTargetCloud(target, target_pose);
    ROS_INFO("preparing target cloud."); 
    icp_.prepareTargetCloud();
  }else{
    ROS_ERROR("Could not get the cloud pose in the map frame, ignoring the target cloud!");
  }
  target_mtx_.unlock();
}

void MapOdomICP::updateMapOdomTransform(geometry_msgs::Transform& delta_transform){

    // convert to tf
    tf::Transform delta_transform_tf;
    tf::transformMsgToTF(delta_transform, delta_transform_tf);

    // calculate the new map to odom_combined transformation
    tf::StampedTransform map_to_odom;
    Sync::getTransform(map_to_odom);
    tf::Transform update_transform_tf = map_to_odom * delta_transform_tf;   

    tf::StampedTransform update_transform_stamped_tf(
      update_transform_tf, ros::Time::now() , "map", "odom_combined");
    Sync::setTransform(update_transform_stamped_tf);

}

void MapOdomICP::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  
  geometry_msgs::PoseStamped result_pose;
  if(!getPointCloudPose(*cloud, "map", cloud_pose_)){
    ROS_ERROR("Could not get the cloud pose in the map frame, ignoring the cloud!");
    return;
  }
  result_pose = cloud_pose_; // init
  
  bool succeeded = true;
  
  if(use_target_cloud_){
    if(icp_.isTargetCloudPrepared()){
      target_mtx_.lock();
      succeeded = icpWithTargetCloud(cloud, cloud_pose_, result_pose);
      target_mtx_.unlock();
    }else{
      ROS_INFO_DELAYED_THROTTLE(5, "No target cloud set");
    }
  }
  else{
    if(previous_cloud_ != 0) {
      target_mtx_.lock();
      icp_.setTargetCloud(previous_cloud_, previous_pose_);
      icp_.prepareTargetCloud();
      succeeded = icpWithTargetCloud(cloud, cloud_pose_, result_pose);    
      target_mtx_.unlock();
    }
  }

  previous_cloud_ = cloud;
  
  if(succeeded){
    previous_pose_ = result_pose; // save only if valid update
    Sync::setUpdated();
  }
}

bool MapOdomICP::icpWithTargetCloud(
  const sensor_msgs::PointCloud2::ConstPtr& cloud,
  const geometry_msgs::PoseStamped& cloud_pose,
  geometry_msgs::PoseStamped& result_pose)
{
  bool succeeded = true;

  geometry_msgs::Transform delta_transform;

  // register point cloud to the target cloud with icp
  ROS_DEBUG("Start to register point cloud to the target cloud.");
  succeeded = icp_.registerCloud(
    *cloud,
    cloud_pose,
    result_pose,
    delta_transform);

  if(succeeded){
    ROS_INFO("Registering point cloud to the target cloud succeeded."); 
    ROS_INFO("Updating the Map to Odom Transformation"); 
    updateMapOdomTransform(delta_transform);
  }
  else{
    ROS_INFO("Registering point cloud to the target cloud failed!");
  }
  return succeeded;
}

bool MapOdomICP::icpWithPreviousCloud(
  const sensor_msgs::PointCloud2::ConstPtr& cloud,
  const geometry_msgs::PoseStamped& cloud_pose,
  geometry_msgs::PoseStamped& result_pose)
{
  ROS_INFO("ICP with previous cloud");
  return icpUpdateMapToOdomCombined(
    previous_cloud_,
    previous_pose_,
    cloud,
    cloud_pose,
    result_pose
  );
}

bool MapOdomICP::icpUpdateMapToOdomCombined(
    const sensor_msgs::PointCloud2::ConstPtr& target,
    const geometry_msgs::PoseStamped& target_pose,
    const sensor_msgs::PointCloud2::ConstPtr& cloud,
    const geometry_msgs::PoseStamped& cloud_pose,
    geometry_msgs::PoseStamped& result_pose
){
  bool succeeded = true;
  ROS_INFO("ICP Update Map to odomCombined");
  ROS_INFO("Received new point cloud");

  geometry_msgs::Transform delta_transform;

  // register point clouds with icp
  ROS_INFO("register point clouds...");
  succeeded = icp_.registerClouds(
    *previous_cloud_,
    previous_pose_,
    *cloud,
    cloud_pose,
    result_pose,
    delta_transform,
    correspondence_distance_,
    transformation_epsilon_,
    maximum_iterations_,
    downsample_target_,
    downsample_cloud_,
    downsample_leafsize_);

  if(succeeded){
    ROS_INFO("Registering point cloud to the previous cloud succeeded."); 
    ROS_INFO("Updating the Map to Odom Transformation"); 
    updateMapOdomTransform(delta_transform);
  }
  else{
    ROS_INFO("Registering point cloud to the previous cloud failed!");
  }
  return succeeded;
}

void MapOdomICP::sendMapToOdomCombined(){

  ros::Time now = ros::Time::now();

  if (Sync::hasUpdated()){
    Sync::getTransform(map_to_odom_);

    if(publish_cloud_ && previous_cloud_ ){
      tf::Stamped<tf::Pose> cloud_pose_tf;
      tf::poseStampedMsgToTF(cloud_pose_, cloud_pose_tf);
      
      tf::Stamped<tf::Pose> result_pose_tf;
      tf::poseStampedMsgToTF(previous_pose_, result_pose_tf);
    
      tf::Transform cloud_pose_tf_inv;
      cloud_pose_tf_inv = cloud_pose_tf.inverse();
      tf::Transform map_odom = result_pose_tf * cloud_pose_tf_inv; 

      tf::StampedTransform update_transform_stamped_tf(map_odom, now, "map", "odom_combined");
      map_to_odom_ = update_transform_stamped_tf;

      ROS_INFO("Transform point cloud to /map frame.");

      pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointNormal>);
      pcl::fromROSMsg<pcl::PointNormal>(*previous_cloud_, *pcl_cloud);
      
      pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud_transformed(new pcl::PointCloud<pcl::PointNormal>);
      pcl_cloud_transformed->header.stamp = pcl_conversions::toPCL(now);
      pcl_cloud_transformed->header.frame_id = "map";
      pcl_ros::transformPointCloudWithNormals(*pcl_cloud, *pcl_cloud_transformed, result_pose_tf);

      sensor_msgs::PointCloud2 cloud_transformed;
      pcl::toROSMsg<pcl::PointNormal>(*pcl_cloud_transformed, cloud_transformed);

      cloud_transformed.header.frame_id = "map";
      cloud_transformed.header.stamp = now;
      ROS_INFO("Publish corrected point cloud.");
      cloud_pub_.publish(cloud_transformed);
    }

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


  }

  tf_broadcaster_.sendTransform(
    tf::StampedTransform(
      map_to_odom_,
      now,
      "map",
      "odom_combined"
    )
  );
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
