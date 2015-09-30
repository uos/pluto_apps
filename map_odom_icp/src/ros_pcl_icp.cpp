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
#include <pcl/features/normal_3d.h>


RosPclIcp::RosPclIcp(ros::NodeHandle &nh)
 :  nh_(nh),
  m_icp_type(1),
 // m_target_xyz_ptr(0),
 // m_target_xyz_normals_ptr(0),
  m_correspondence_distance(0.1),
  m_downsample_target(false),
  m_downsample_source(false),
  m_downsample_leafsize(0.04),
  m_maximum_iterations(100),
  m_transformation_epsilon(10e-8),
  m_maximum_rejection_distance(0.4),
  m_maximum_rejection_angle(3.14/4),
  m_target_cloud_set(false)

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

bool RosPclIcp::isTargetCloudSet(){
  return m_target_cloud_set;
}

void RosPclIcp::setTargetCloud(
  const sensor_msgs::PointCloud2::ConstPtr& target_cloud,
  geometry_msgs::PoseStamped &target_pose
){
  // create shared pointers
  m_target_xyz_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  m_target_ptr = target_cloud;
  convertPointCloud2ToPcl(*target_cloud, *m_target_xyz_ptr);
  m_target_pose = target_pose; 
  m_target_cloud_set = true;
}

void RosPclIcp::setIcpType(int type){
  switch(type){
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    default:
      ROS_ERROR("unknown icp type selected!");
      return;
  }
}

void RosPclIcp::setCorrespondenceDistance(double dist){
  m_correspondence_distance = dist;
}

void RosPclIcp::setDownsampleTarget(bool downsample_target){
  m_downsample_target = downsample_target;
}

void RosPclIcp::setDownsampleSource(bool downsample_source){
  m_downsample_source = downsample_source;
}

void RosPclIcp::setDownsampleLeafSize(double leafsize){
  m_downsample_leafsize = leafsize;
}

void RosPclIcp::setMaximumIterations(int iterations){
  m_maximum_iterations = iterations;
}

void RosPclIcp::setTransformationEpsilon(double epsilon){
  m_transformation_epsilon = epsilon;
}

void RosPclIcp::setMaximumRejectionDistance(double dist){
  m_maximum_rejection_distance = dist;
}

void RosPclIcp::setMaximumRejectionAngle(double angle){
  m_maximum_rejection_angle = angle;
}

void RosPclIcp::convertPointCloud2ToPcl(
  const sensor_msgs::PointCloud2& cloud, 
  pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_xyz){

  // convert point clouds ro pcl
  pcl::PCLPointCloud2 pcl_cloud2;
  pcl_conversions::toPCL(cloud, pcl_cloud2);
  pcl::fromPCLPointCloud2(pcl_cloud2, pcl_cloud_xyz);

}

void RosPclIcp::prepareTarget(){

  // downsample target
  if(m_downsample_target){
    ROS_INFO("Downsampling target cloud.");
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_downsampled_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    downsampleCloud(m_target_xyz_ptr, target_xyz_downsampled_ptr, m_downsample_leafsize);
    m_target_xyz_ptr = target_xyz_downsampled_ptr;
  }

  // estimate normals
  ROS_INFO("Estimating normals for the target cloud.");
  m_target_xyz_normals_ptr = 
    pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
  estimateNormals(m_target_xyz_ptr, m_target_xyz_normals_ptr, 0.08f);
  pcl::copyPointCloud (*m_target_xyz_ptr, *m_target_xyz_normals_ptr);
}

bool RosPclIcp::registerCloud(
  const sensor_msgs::PointCloud2& source,
  const geometry_msgs::PoseStamped& source_pose,
  geometry_msgs::PoseStamped &result_pose,
  geometry_msgs::Transform &delta_transform
){
  // calculate initial transform
  tf::Stamped<tf::Pose> target_pose_tf;
  tf::Stamped<tf::Pose> source_pose_tf;
  tf::poseStampedMsgToTF(m_target_pose, target_pose_tf);
  tf::poseStampedMsgToTF(source_pose, source_pose_tf);
  // calculate transformation between target_pose and cloud_pose
  tf::Transform initial_transform_tf = target_pose_tf.inverseTimes(source_pose_tf);
  Eigen::Matrix4f initial_transform;
  tfToEigen(initial_transform_tf, initial_transform);
  
  // create shared pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // convert point cloud to pcl
  convertPointCloud2ToPcl(source, *source_xyz_ptr);

  // downsample source
  if(m_downsample_source){
    ROS_INFO("Downsampling the source cloud.");
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz_downsampled_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    downsampleCloud(source_xyz_ptr, source_xyz_downsampled_ptr, m_downsample_leafsize);
    source_xyz_ptr = source_xyz_downsampled_ptr;
  }

  // estimate normals
  // create shared pointer
  pcl::PointCloud<pcl::PointNormal>::Ptr source_xyz_normals_ptr(new pcl::PointCloud<pcl::PointNormal>);
  
  estimateNormals(source_xyz_ptr, source_xyz_normals_ptr, 0.08f);
  pcl::copyPointCloud (*source_xyz_ptr, *source_xyz_normals_ptr);

  // remove nan values from point clouds
  std::vector<int> indices_cloud;

  indices_cloud.clear();
  pcl::removeNaNFromPointCloud(*source_xyz_normals_ptr, *source_xyz_normals_ptr, indices_cloud);
  indices_cloud.clear();
  pcl::removeNaNFromPointCloud(*m_target_xyz_normals_ptr, *m_target_xyz_normals_ptr, indices_cloud);
  
  indices_cloud.clear();
  pcl::removeNaNFromPointCloud(*source_xyz_ptr, *source_xyz_ptr, indices_cloud);
  indices_cloud.clear();
  pcl::removeNaNFromPointCloud(*m_target_xyz_ptr, *m_target_xyz_ptr, indices_cloud);

  ROS_INFO("start icp...");
  // do icp
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;
  //pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal, float> icp;

  icp.setMaxCorrespondenceDistance(m_correspondence_distance);
  icp.setMaximumIterations (m_maximum_iterations);
  icp.setTransformationEpsilon (m_transformation_epsilon);
  
  //icp.setInputSource(source_xyz_normals_ptr);
  //icp.setInputTarget(m_target_xyz_normals_ptr);
 
  icp.setInputSource(source_xyz_ptr);
  icp.setInputTarget(m_target_xyz_ptr);

  //pcl::PointCloud<pcl::PointNormal> result;
  pcl::PointCloud<pcl::PointXYZ> result;
  icp.align(result, initial_transform);

  ROS_INFO("ICP has converged: %s, score: %f", icp.hasConverged()? "true":
  "false", icp.getFitnessScore());

  //std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f final_transform = icp.getFinalTransformation();

  tf::Transform final_transform_tf;
  eigenToTf(final_transform, final_transform_tf);

  // combine the target_pose and the icp transformation 
  // between the clouds to get the global pose
  tf::Transform result_pose_tf = target_pose_tf * final_transform_tf;
  tf::Stamped<tf::Pose> result_pose_stamped_tf(result_pose_tf, ros::Time::now(), source_pose.header.frame_id);
  // convert to pose msg
  tf::poseStampedTFToMsg(result_pose_stamped_tf, result_pose);
  
  tf::Transform delta_transform_tf = initial_transform_tf.inverseTimes(final_transform_tf);
  
  tf::Vector3 origin = delta_transform_tf.getOrigin();
  tf::Matrix3x3 basis = delta_transform_tf.getBasis();
  
  tfScalar roll, pitch, yaw;
  basis.getRPY(roll, pitch, yaw);
  
  bool reject_dist = m_maximum_rejection_distance < origin.length();
  bool reject_angle = 
    m_maximum_rejection_angle < roll ||
    m_maximum_rejection_angle < pitch ||
    m_maximum_rejection_angle < yaw;

  if(reject_dist){
    ROS_INFO("Reject delta transform, the distance is to big!");
  }
  if(reject_angle){
    ROS_INFO("Reject delta transform, the angle is to big!");
  }

  tf::transformTFToMsg(delta_transform_tf, delta_transform);

  return icp.hasConverged() && !reject_dist && !reject_angle;
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

  // calculate initial transform
  tf::Stamped<tf::Pose> target_pose_tf;
  tf::Stamped<tf::Pose> cloud_pose_tf;
  tf::poseStampedMsgToTF(target_pose, target_pose_tf);
  tf::poseStampedMsgToTF(cloud_pose, cloud_pose_tf);
  // calculate transformation between target_pose and cloud_pose
  tf::Transform initial_transform_tf = target_pose_tf.inverseTimes(cloud_pose_tf);
  Eigen::Matrix4f initial_transform;
  tfToEigen(initial_transform_tf, initial_transform);
  
  
  // create shared pointers
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // convert point clouds to pcl
  convertPointCloud2ToPcl(target, *target_xyz_ptr);
  convertPointCloud2ToPcl(cloud, *cloud_xyz_ptr);

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

  // estimate normals
  // create shared pointers
  pcl::PointCloud<pcl::PointNormal>::Ptr target_xyz_normals_ptr(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyz_normals_ptr(new pcl::PointCloud<pcl::PointNormal>);
  
  estimateNormals(target_xyz_ptr, target_xyz_normals_ptr, 0.08f);
  pcl::copyPointCloud (*target_xyz_ptr, *target_xyz_normals_ptr);
  estimateNormals(cloud_xyz_ptr, cloud_xyz_normals_ptr, 0.08f);
  pcl::copyPointCloud (*cloud_xyz_ptr, *cloud_xyz_normals_ptr);

  // do icp
  //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal, float> icp;

  icp.setMaxCorrespondenceDistance(correspondence_distance);
  icp.setMaximumIterations (maximum_iterations);
  icp.setTransformationEpsilon (transformation_epsilon);
  
  icp.setInputSource(cloud_xyz_normals_ptr);
  icp.setInputTarget(target_xyz_normals_ptr);
  
  pcl::PointCloud<pcl::PointNormal> result;
  icp.align(result, initial_transform);

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
  
  tf::Transform delta_transform_tf = initial_transform_tf.inverseTimes(final_transform_tf);
  tf::transformTFToMsg(delta_transform_tf, delta_transform);

  return icp.hasConverged();
}


void RosPclIcp::estimateNormals(
   pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_in, 
   pcl::PointCloud<pcl::PointNormal>::Ptr xyz_normals_out,
   float radius)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  //norm_est.setKSearch (30);
  norm_est.setRadiusSearch(radius);

  norm_est.setInputCloud (xyz_in);
  norm_est.compute (*xyz_normals_out);
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

