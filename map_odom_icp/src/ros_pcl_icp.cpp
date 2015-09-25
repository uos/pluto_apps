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
 *  ros_pcl_icp.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */



#include <ros_pcl_icp.h>

RosPclIcp::RosPclIcp(ros::NodeHandle &nh)
 :  nh_(nh)
{
  service = nh_.advertiseService("icp", &RosPclIcp::registerCloudsSrv, this);
}

bool RosPclIcp::registerCloudsSrv( map_odom_icp::IcpSrv::Request &req,
                                  map_odom_icp::IcpSrv::Response &res){

  return registerClouds(req.target,
                        req.target_pose,
                        req.cloud,
                        req.cloud_pose,
                        res.result_pose,
                        res.delta_transform,
                        req.correspondence_distance);
}

bool RosPclIcp::registerClouds(
  const sensor_msgs::PointCloud2 &target,
  const geometry_msgs::PoseStamped &target_pose,
  const sensor_msgs::PointCloud2 &cloud,
  const geometry_msgs::PoseStamped &cloud_pose,
  geometry_msgs::PoseStamped &result_pose,
  geometry_msgs::Transform &delta_transform,
  double correspondence_distance,
  double transformation_epsilon,
  int maximum_iterations,
  bool downsample_target,
  bool downsample_cloud,
  float downsample_leafsize
  )
{

  // create shared pointers
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
  
  tf::Stamped<tf::Pose> target_pose_tf;
  tf::Stamped<tf::Pose> cloud_pose_tf;

  tf::poseStampedMsgToTF(target_pose, target_pose_tf);
  tf::poseStampedMsgToTF(cloud_pose, cloud_pose_tf);

  // calculate transformation between target_pose and cloud_pose
  tf::Transform guess_transform_tf = target_pose_tf.inverseTimes(cloud_pose_tf);
  //tf::Transform guess_transform_tf = cloud_pose_tf.inverseTimes(target_pose_tf);
  
  Eigen::Matrix4f guess_transform;
  tfToEigen(guess_transform_tf, guess_transform);

  //std::cout << guess_transform << std::endl;

  // convert point clouds ro pcl
  pcl::PCLPointCloud2 pcl_target, pcl_cloud;
  pcl_conversions::toPCL(target, pcl_target);
  pcl_conversions::toPCL(cloud, pcl_cloud);
  pcl::fromPCLPointCloud2(pcl_target, *target_xyz_ptr);
  pcl::fromPCLPointCloud2(pcl_cloud, *cloud_xyz_ptr);

  // remove nan values from point clouds
  std::vector<int> indices_target;
  std::vector<int> indices_cloud;
  pcl::removeNaNFromPointCloud(*target_xyz_ptr,*target_xyz_ptr, indices_target);
  pcl::removeNaNFromPointCloud(*cloud_xyz_ptr,*cloud_xyz_ptr, indices_cloud);

  // downsample clouds
  if(downsample_target){
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_downsampled_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    downsampleCloud(target_xyz_ptr, target_xyz_downsampled_ptr, downsample_leafsize);
    target_xyz_ptr = target_xyz_downsampled_ptr;
  }
  if(downsample_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_downsampled_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    downsampleCloud(cloud_xyz_ptr, cloud_xyz_downsampled_ptr, downsample_leafsize);
    cloud_xyz_ptr = cloud_xyz_downsampled_ptr;
  }

  // do icp
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;

  icp.setMaxCorrespondenceDistance(correspondence_distance);
  icp.setMaximumIterations (maximum_iterations);
  icp.setTransformationEpsilon (transformation_epsilon);
  
  icp.setInputSource(cloud_xyz_ptr);
  icp.setInputTarget(target_xyz_ptr);
  
  pcl::PointCloud<pcl::PointXYZ> result;
  icp.align(result, guess_transform);

  ROS_INFO("ICP has converged: %s, score: %f", icp.hasConverged()? "true":
  "false", icp.getFitnessScore());

  //std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f final_transform = icp.getFinalTransformation();

  tf::Transform final_transform_tf;
  eigenToTf(final_transform, final_transform_tf);

  // combine the target_pose and the icp transformation 
  // between the clouds to get the global pose
  tf::Transform result_pose_tf = target_pose_tf * final_transform_tf;
  tf::Stamped<tf::Pose> result_pose_stamped_tf(result_pose_tf, ros::Time::now(), cloud_pose.header.frame_id);
  // convert to pose msg
  tf::poseStampedTFToMsg(result_pose_stamped_tf, result_pose);
  
  tf::Transform delta_transform_tf = guess_transform_tf.inverseTimes(final_transform_tf);
  tf::transformTFToMsg(delta_transform_tf, delta_transform);

  return icp.hasConverged();
}


void RosPclIcp::downsampleCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
    float leafSize)
{
  pcl::VoxelGrid<pcl::PointXYZ>sor;
  sor.setInputCloud (input);
  sor.setLeafSize (leafSize, leafSize, leafSize);
  sor.filter(*output);
}


void RosPclIcp::eigenToTf(
    const Eigen::Matrix4f &transform_eigen,
    tf::Transform &transform_tf){
  
  //create an affine transformation as helper object
  Eigen::Affine3f affine(transform_eigen);
  Eigen::Quaternionf q = (Eigen::Quaternionf)affine.linear();
  
  //create tf transform object
  tf::Vector3 origin;
  origin.setX(affine.translation()[0]);
  origin.setY(affine.translation()[1]);
  origin.setZ(affine.translation()[2]);
  tf::Quaternion rotation;
  if(q.w() > 0){
    rotation.setX(q.x());
    rotation.setY(q.y());
    rotation.setZ(q.z());
    rotation.setW(q.w());
  }else{
    rotation.setX(-q.x());
    rotation.setY(-q.y());
    rotation.setZ(-q.z());
    rotation.setW(-q.w()); 
  }
  transform_tf.setOrigin(origin);
  transform_tf.setRotation(rotation);
}

void RosPclIcp::tfToEigen(
  const tf::Transform &transform_tf,
  Eigen::Matrix4f &transform_eigen)
{
  tf::Vector3 translation = transform_tf.getOrigin();
  tf::Quaternion rotation = transform_tf.getRotation();

  Eigen::Affine3f affine =
    Eigen::Translation3f( translation.getX(),
                          translation.getY(),
                          translation.getZ()) *
    Eigen::Quaternionf( rotation.getW(),
                        rotation.getX(),
                        rotation.getY(),
                        rotation.getZ()); 
  transform_eigen = affine.matrix();
}

