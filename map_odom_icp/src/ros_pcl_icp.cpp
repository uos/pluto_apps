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
#include <pcl/common/transforms.h>

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
  m_target_cloud_set(false),
  m_target_cloud_prepared(false),
  m_viewer(0),
  m_viewer_enabled(false)
{
  service = nh_.advertiseService("icp", &RosPclIcp::registerCloudsSrv, this);
  ros::NodeHandle p_nh("~");
// viewer 
  p_nh.param("show_viewer", m_viewer_enabled, false);
}

RosPclIcp::~RosPclIcp(){
  showViewer(false);
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


void RosPclIcp::startViewer(){
  m_viewer_mtx.lock();
  // only if the state changes start or stop the viewer thread
  if(m_viewer_enabled && !m_viewer){
    ROS_INFO("Start viewer...");
    m_viewer = new pcl::visualization::PCLVisualizer("ros_icp_clouds");
    m_viewer_thread = boost::thread(boost::bind(&RosPclIcp::viewerLoop, this));
  }
  m_viewer_mtx.unlock();
}

void RosPclIcp::showViewer(bool enable){
  // only if the state changes stop the viewer thread
  ROS_INFO("show viewer to false!");
  if(m_viewer_enabled && !enable){
    m_viewer_thread.interrupt();
    m_viewer_thread.join();
  }
  m_viewer_enabled = enable;
}

void RosPclIcp::viewerLoop(){

  bool run = true;
  while(run){
    try{
      m_viewer_mtx.lock();
      // viewer
      if ( m_viewer->wasStopped() ) { // Display the visualiser until 'q' key is pressed
        m_viewer->close();
        delete m_viewer;
        m_viewer = 0;
        run = false;
      }else{
        m_viewer->spinOnce ();
      }
      m_viewer_mtx.unlock();
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000/30));
    }catch(boost::thread_interrupted&){
      m_viewer_mtx.lock();
      m_viewer->close();
      delete m_viewer;
      m_viewer = 0;
      run = false;
      m_viewer_mtx.unlock();
    }
  }
}

bool RosPclIcp::isTargetCloudSet(){
  return m_target_cloud_set;
}

void RosPclIcp::setTargetCloud(
  const sensor_msgs::PointCloud2::ConstPtr& target_cloud,
  geometry_msgs::PoseStamped &target_pose
){
  m_target_cloud_prepared = false;
  m_target_ptr = target_cloud;
  m_target_pose = target_pose; 
  m_target_cloud_set = true;
}

void RosPclIcp::setIcpType(int type){
  switch(type){
    case 0:
    case 1:
      m_icp_type = type;
      break;
    default:
      ROS_ERROR("Unknown ICP type selected! Continue with ICP point to point");
      m_icp_type = ICP_TYPE_POINT_TO_POINT;
      break;
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


bool RosPclIcp::isTargetCloudPrepared(){
  return m_target_cloud_prepared;
}

void RosPclIcp::prepareTargetCloud(){
  m_target_cloud_prepared = false;
  
  bool target_has_normals = false;
  for (size_t i = 0; i < m_target_ptr->fields.size (); ++i){
    if (m_target_ptr->fields[i].name == "normal_x"){
      target_has_normals = true;
      ROS_INFO("target already has normals, using the given normals");
      break;
    }
  }

  m_target_xyz_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  m_target_xyz_normals_ptr = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
  
  if(target_has_normals){
    convertPointCloud2ToPcl<pcl::PointNormal>(*m_target_ptr, *m_target_xyz_normals_ptr);
    pcl::copyPointCloud (*m_target_xyz_normals_ptr, *m_target_xyz_ptr);
  }
  else{
    convertPointCloud2ToPcl<pcl::PointXYZ>(*m_target_ptr, *m_target_xyz_ptr);
    
    // estimate normals
    ROS_INFO("Estimating normals for the target cloud.");
    estimateNormals<pcl::PointXYZ, pcl::PointNormal>(m_target_xyz_ptr, m_target_xyz_normals_ptr, 0.4f);
    pcl::copyPointCloud (*m_target_xyz_ptr, *m_target_xyz_normals_ptr);
    ROS_INFO("Target cloud size after normals estimation: %d", m_target_xyz_normals_ptr->size() );
  
  }

  // remove point NaN values
  std::vector<int> indices_target;
  pcl::removeNaNFromPointCloud(*m_target_xyz_normals_ptr, *m_target_xyz_normals_ptr, indices_target);
  ROS_INFO("Target cloud size after removing NaN point values: %d", m_target_xyz_normals_ptr->size() );
  // remove normal NaN values
  std::vector<int> indices_normals_target;
  pcl::removeNaNNormalsFromPointCloud(*m_target_xyz_normals_ptr, *m_target_xyz_normals_ptr, indices_normals_target);
  ROS_INFO("Target cloud size after removing NaN normal values: %d", m_target_xyz_normals_ptr->size() );
 
  ROS_DEBUG("Input target cloud size: %d", m_target_xyz_ptr->size() );
  ROS_DEBUG("Downsample target cloud with leafsize: %f", m_downsample_leafsize);
  // downsample target
  if(m_downsample_target){
    ROS_INFO("Downsampling target cloud.");
    pcl::PointCloud<pcl::PointNormal>::Ptr target_xyz_normals_downsampled_ptr(
      new pcl::PointCloud<pcl::PointNormal>);
    downsampleCloud<pcl::PointNormal>(m_target_xyz_normals_ptr, target_xyz_normals_downsampled_ptr, m_downsample_leafsize);
    m_target_xyz_normals_ptr = target_xyz_normals_downsampled_ptr;
  }
  ROS_INFO("Target cloud size after downsampling: %d", m_target_xyz_ptr->size() );


  
  m_target_cloud_prepared = true;
}

bool RosPclIcp::registerCloud(
  const sensor_msgs::PointCloud2& source,
  const geometry_msgs::PoseStamped& source_pose,
  geometry_msgs::PoseStamped &result_pose,
  geometry_msgs::Transform &delta_transform
){
  if(!isTargetCloudPrepared()){
    ROS_ERROR("Target cloud is not set, do not execute register cloud!");
    return false;
  }
  
  bool source_has_normals = false;

  for (size_t i = 0; i < source.fields.size (); ++i){
    if (source.fields[i].name == "normal_x"){
      source_has_normals = true;
      ROS_INFO("cloud already has normals, using the given normals");
      break;
    }
  }
  
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
  pcl::PointCloud<pcl::PointNormal>::Ptr source_xyz_normals_ptr(new pcl::PointCloud<pcl::PointNormal>);
  // convert point cloud to pcl
  if(source_has_normals){
    
    convertPointCloud2ToPcl<pcl::PointNormal>(source, *source_xyz_normals_ptr);
    pcl::copyPointCloud (*source_xyz_normals_ptr, *source_xyz_ptr);

  }
  else{
    convertPointCloud2ToPcl<pcl::PointXYZ>(source, *source_xyz_ptr);  
    estimateNormals<pcl::PointXYZ, pcl::PointNormal>(source_xyz_ptr, source_xyz_normals_ptr, 0.3f);
    pcl::copyPointCloud (*source_xyz_ptr, *source_xyz_normals_ptr);
  
  }
  // remove NaN point values
  std::vector<int> indices_source;
  pcl::removeNaNFromPointCloud(*source_xyz_ptr, *source_xyz_ptr, indices_source);
  // remove NaN normal values
  std::vector<int> indices_normals_source;
  pcl::removeNaNNormalsFromPointCloud(*source_xyz_normals_ptr, *source_xyz_normals_ptr, indices_normals_source);
  

  // downsample source
  if(m_downsample_source){
    ROS_INFO("Downsampling the source cloud.");
    pcl::PointCloud<pcl::PointNormal>::Ptr source_xyz_normals_downsampled_ptr(
      new pcl::PointCloud<pcl::PointNormal>);
    downsampleCloud<pcl::PointNormal>(source_xyz_normals_ptr, source_xyz_normals_downsampled_ptr, m_downsample_leafsize);
    source_xyz_normals_ptr = source_xyz_normals_downsampled_ptr;
  }

  Eigen::Matrix4f final_transform;
  bool converged = false; 

  // Point To Point
  if( m_icp_type == ICP_TYPE_POINT_TO_POINT ){
    ROS_INFO("ICP Type With Point to Point selected.");
    // set up icp
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal, float> icp;
    icp.setInputSource(source_xyz_normals_ptr);
    icp.setInputTarget(m_target_xyz_normals_ptr);
    icp.setMaxCorrespondenceDistance(m_correspondence_distance);
    icp.setMaximumIterations (m_maximum_iterations);
    icp.setTransformationEpsilon (m_transformation_epsilon);
    pcl::PointCloud<pcl::PointNormal> result;
    
    // do icp
    icp.align(result, initial_transform);
    // read transform and converge
    final_transform = icp.getFinalTransformation();
    converged = icp.hasConverged();
    ROS_INFO("ICP - Point to Point - has converged: %s, score: %f", icp.hasConverged()? "true":"false", icp.getFitnessScore());
  }
  // Point to Plane
  else if( m_icp_type == ICP_TYPE_WITH_NORMALS )  {
    ROS_INFO("ICP Type With Normals selected.");
    
    // set up icp
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal, float> icp;
    icp.setInputSource(source_xyz_normals_ptr);
    icp.setInputTarget(m_target_xyz_normals_ptr);
    icp.setMaxCorrespondenceDistance(m_correspondence_distance);
    icp.setMaximumIterations (m_maximum_iterations);
    icp.setTransformationEpsilon (m_transformation_epsilon);
    pcl::PointCloud<pcl::PointNormal> result;
    
    // do icp
    icp.align(result, initial_transform);
    final_transform = icp.getFinalTransformation();
    converged = icp.hasConverged(); 
    ROS_INFO("ICP - With Normals - has converged: %s, score: %f", icp.hasConverged()? "true":"false", icp.getFitnessScore());
  }

  tf::Transform final_transform_tf;
  eigenToTf(final_transform, final_transform_tf);

  // combine the target_pose and the icp transformation
  // between the clouds to get the updated global pose of the cloud
  tf::Transform result_pose_tf = final_transform_tf * target_pose_tf;
  //tf::Transform guess_pose_tf = initial_transform_tf * target_pose_tf;
  tf::Stamped<tf::Pose> result_pose_stamped_tf(result_pose_tf, ros::Time::now(), source_pose.header.frame_id);
  // convert to pose msg
  tf::poseStampedTFToMsg(result_pose_stamped_tf, result_pose);
  
  tf::Transform delta_transform_tf = initial_transform_tf.inverseTimes(final_transform_tf);
  //tf::Transform delta_transform_tf = final_transform_tf.inverseTimes(initial_transform_tf);
  //tf::Transform delta_transform_tf = source_pose_tf.inverseTimes(result_pose_tf);
  
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
  
  
  // viewer 
  if(m_viewer_enabled){
    startViewer();

    m_viewer_mtx.lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      source_transformed_xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
        *source_xyz_ptr,
        *source_transformed_xyz_cloud,
        initial_transform
        );

    pcl::PointCloud<pcl::PointXYZ>::Ptr
      source_icp_transformed_xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(
        *source_xyz_ptr,
        *source_icp_transformed_xyz_cloud,
        final_transform
        );

    // Define R,G,B colors for the point cloud
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // source_cloud_color_handler (source_xyz_ptr, 255, 255, 255); //white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      source_transformed_cloud_color_handler (source_transformed_xyz_cloud, 230, 20, 20); // Red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_cloud_color_handler (m_target_xyz_ptr, 20, 20, 230); // Blue
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      source_icp_transformed_cloud_color_handler (source_icp_transformed_xyz_cloud, 20, 230, 20); // Green

    m_viewer->removeAllPointClouds();
    //m_viewer->addPointCloud (source_xyz_ptr, source_cloud_color_handler, "source_cloud");
    m_viewer->addPointCloud (source_transformed_xyz_cloud, source_transformed_cloud_color_handler, "transformed_cloud");
    m_viewer->addPointCloud (m_target_xyz_ptr, target_cloud_color_handler, "target_cloud");
    m_viewer->addPointCloud (source_icp_transformed_xyz_cloud, source_icp_transformed_cloud_color_handler, "icp_cloud");

    //viewer.addCoordinateSystem (1.0, "cloud", 0);
    m_viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // m_viewer->setPointCloudRenderingProperties
    //  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud");
    m_viewer->setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    m_viewer->setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
    m_viewer->setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_cloud");
    //m_viewer.setPosition(800, 400); // Setting visualiser window position
    m_viewer_mtx.unlock();
  }

  return converged && !reject_dist && !reject_angle;
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


  int icp_type = ICP_TYPE_WITH_NORMALS;

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
  pcl::PointCloud<pcl::PointNormal>::Ptr target_xyz_ptr(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointNormal>);

  bool cloud_has_normals = false;
  bool target_has_normals = false;

  for (size_t i = 0; i < cloud.fields.size (); ++i){
    if (cloud.fields[i].name == "normal_x"){
      cloud_has_normals = true;
      ROS_INFO("cloud already has normals, using the given normals");
      break;
    }
  }
  
  for (size_t i = 0; i < target.fields.size (); ++i){
    if (target.fields[i].name == "normal_x"){
      target_has_normals = true;
      ROS_INFO("target already has normals, using the given normals");
      break;
    }
  }


  // convert point clouds to pcl
  convertPointCloud2ToPcl<pcl::PointNormal>(target, *target_xyz_ptr);
  convertPointCloud2ToPcl<pcl::PointNormal>(cloud, *cloud_xyz_ptr);

  // downsample clouds
  if(downsample_target){
    pcl::PointCloud<pcl::PointNormal>::Ptr target_xyz_downsampled_ptr(
      new pcl::PointCloud<pcl::PointNormal>);
    downsampleCloud<pcl::PointNormal>(target_xyz_ptr, target_xyz_downsampled_ptr, downsample_leafsize);
    target_xyz_ptr = target_xyz_downsampled_ptr;
  }
  if(downsample_cloud){
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyz_downsampled_ptr(
      new pcl::PointCloud<pcl::PointNormal>);
    downsampleCloud<pcl::PointNormal>(cloud_xyz_ptr, cloud_xyz_downsampled_ptr, downsample_leafsize);
    cloud_xyz_ptr = cloud_xyz_downsampled_ptr;
  }

  // estimate normals
  // create shared pointers
  if(!target_has_normals){
    ROS_INFO("The target has no normals, estimating normals!");
    estimateNormals<pcl::PointNormal, pcl::PointNormal>(target_xyz_ptr, target_xyz_ptr, 0.08f);
  }
  if(!cloud_has_normals){ 
    ROS_INFO("The cloud has no normals, estimating normals!");
    estimateNormals<pcl::PointNormal, pcl::PointNormal>(cloud_xyz_ptr, cloud_xyz_ptr, 0.08f);
  }
  Eigen::Matrix4f final_transform;
  bool converged = false; 
  // Point To Point
  if( icp_type == ICP_TYPE_POINT_TO_POINT ){
    ROS_INFO("ICP Type With Point to Point selected.");
    // set up icp
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal, float> icp;
    icp.setInputSource(cloud_xyz_ptr);
    icp.setInputTarget(target_xyz_ptr);
    icp.setMaxCorrespondenceDistance(correspondence_distance);
    icp.setMaximumIterations (maximum_iterations);
    icp.setTransformationEpsilon (transformation_epsilon);
    pcl::PointCloud<pcl::PointNormal> result;

    // do icp
    icp.align(result, initial_transform);
    // read transform and converge
    final_transform = icp.getFinalTransformation();
    converged = icp.hasConverged();
    ROS_INFO("ICP - Point to Point - has converged: %s, score: %f", icp.hasConverged()? "true":"false", icp.getFitnessScore());
  }
  // Point to Plane
  else if( icp_type == ICP_TYPE_WITH_NORMALS )  {
    ROS_INFO("ICP Type With Normals selected.");
    // set up icp
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal, float> icp;
    icp.setInputSource(cloud_xyz_ptr);
    icp.setInputTarget(target_xyz_ptr);
    icp.setMaxCorrespondenceDistance(correspondence_distance);
    icp.setMaximumIterations (maximum_iterations);
    icp.setTransformationEpsilon (transformation_epsilon);
    pcl::PointCloud<pcl::PointNormal> result;
    
    // do icp
    icp.align(result, initial_transform);
    final_transform = icp.getFinalTransformation();
    converged = icp.hasConverged(); 
    ROS_INFO("ICP - With Normals - has converged: %s, score: %f", icp.hasConverged()? "true":"false", icp.getFitnessScore());
  }else{
    ROS_ERROR("unsupported icp type selected! - cancel");
    return false;
  }

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

  return converged;
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

