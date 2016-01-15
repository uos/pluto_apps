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
 *  organized_point_normals.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "organized_point_normals/organized_point_normals.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>





OrganizedPointNormals::OrganizedPointNormals(ros::NodeHandle& nh)
  : viewer(0),
    nh(nh),
    max_depth_change_factor(0.1),
    normal_smoothing_size(1),
    normal_estimation_mode(0)
{
  sub = nh.subscribe("organized_cloud", 1, &OrganizedPointNormals::pointCloudCallback, this);
  pub = nh.advertise<sensor_msgs::PointCloud2>("organized_cloud_with_normals", 1);
  
  ros::NodeHandle p_nh("~");
  p_nh.param("normal_estimation_mode", normal_estimation_mode, 0);
  p_nh.param("normal_smoothing_size", normal_smoothing_size, 1.0);
  p_nh.param("max_depth_change_factor", max_depth_change_factor, 0.1);
  p_nh.param("show_viewer", show_viewer, false);

  dynamic_reconfigure::Server<organized_point_normals::OrganizedPointNormalsConfig>::CallbackType f;
  f = boost::bind(&OrganizedPointNormals::reconfigureCallback, this, _1, _2);
  reconfigure_server.setCallback(f);
  
}
OrganizedPointNormals::~OrganizedPointNormals(){

}

void OrganizedPointNormals::reconfigureCallback(organized_point_normals::OrganizedPointNormalsConfig& config, uint32_t level){
  normal_estimation_mode = config.normal_estimation_mode;
  normal_smoothing_size = config.normal_smoothing_size;
  max_depth_change_factor = config.max_depth_change_factor;
  

}

void OrganizedPointNormals::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr){
  // convert to pcl PointCloud
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*cloud_ptr, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_organized (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_cloud, *cloud_organized);

  if(!cloud_organized->isOrganized()){
    ROS_WARN("The given cloud is not organized, ignoring the cloud!");
    return;
  }

  // estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  if(normal_estimation_mode == NE_MODE_ORGANIZED_FAST_NORMALS){
    ROS_INFO("NE_MODE_ORGANIZED_FAST_NORMALS");
    calculateOrganizedFastNormals(cloud_organized, normals);
  }else if( normal_estimation_mode == NE_MODE_COVARIANCE_MATRIX || 
            normal_estimation_mode == NE_MODE_AVERAGE_3D_GRADIENT ||
            normal_estimation_mode == NE_MODE_AVERAGE_DEPTH_CHANGE ||
            normal_estimation_mode == NE_MODE_SIMPLE_3D_GRADIENT){
      pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod mode;
      switch(normal_estimation_mode){
        case NE_MODE_COVARIANCE_MATRIX:
          ROS_INFO("NE_MODE_COVARIANCE_MATRIX");
          mode = ne.COVARIANCE_MATRIX;
          break;
        case NE_MODE_AVERAGE_3D_GRADIENT:
          ROS_INFO("NE_MODE_AVERAGE_3D_GRADIENT");
          mode = ne.AVERAGE_3D_GRADIENT;
          break;
        case NE_MODE_AVERAGE_DEPTH_CHANGE:
          ROS_INFO("NE_MODE_AVERAGE_DEPTH_CHANGE");
          mode = ne.AVERAGE_DEPTH_CHANGE;
          break;
        case NE_MODE_SIMPLE_3D_GRADIENT:
          ROS_INFO("NE_MODE_SIMPLE_3D_GRADIENT");
          mode = ne.SIMPLE_3D_GRADIENT;
          break;
      }

      ne.setNormalEstimationMethod(mode);
      ne.setMaxDepthChangeFactor(max_depth_change_factor);  // 0.02f
      ne.setNormalSmoothingSize(normal_smoothing_size);  //10
      ne.setInputCloud(cloud_organized);
      ne.setBorderPolicy( pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::BORDER_POLICY_MIRROR); 

      ne.compute(*normals);
  }else{
    ROS_ERROR("unknown normal estimation mode!");
    return;
  }

  // concatenate clouds
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_organized_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_organized, *normals, *cloud_organized_normals);
  
  // visualize normals
  if(show_viewer){
    if(!viewer){
      viewer = new pcl::visualization::PCLVisualizer("Estimated Normals");
      viewer->setBackgroundColor (0.0, 0.0, 0.5);
    }
    viewer->removeAllPointClouds();
    viewer->addPointCloudNormals<pcl::PointNormal>(cloud_organized_normals);
  }

  // convert to PointCloud2 and publish
  pcl::PCLPointCloud2 output;
  pcl::toPCLPointCloud2 (*cloud_organized_normals, output);
  sensor_msgs::PointCloud2 pc2_output;
  pcl_conversions::fromPCL(output, pc2_output);
  pub.publish(pc2_output);

  // use the old cloud time stamp
  pc2_output.header.stamp = cloud_ptr->header.stamp;
}

void OrganizedPointNormals::calculateOrganizedFastNormals(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud,
  pcl::PointCloud<pcl::Normal>::Ptr normals){

  uint32_t width = organized_cloud->width;
  uint32_t height = organized_cloud->height;
  size_t normals_cnt = 0;

  normals->width = width;
  normals->height = height;

  normals->points.resize(width*height);

  for(uint32_t x=0; x<width; x++){
    for(uint32_t y=0; y<height; y++){
      Eigen::Vector3f self = (*organized_cloud)(x, y).getVector4fMap().head<3>();
      if(!self.allFinite()){
        (*normals)(x, y).normal[0] = nan("");
        (*normals)(x, y).normal[1] = nan("");
        (*normals)(x, y).normal[2] = nan("");
        continue;
      }
      
      uint32_t x_left = (x == 0) ? width-1 : x-1;
      uint32_t x_right = (x == width-1) ? 0 : x+1;
      uint32_t y_top = (y == 0) ? height-1 : y-1;
      uint32_t y_bottom = (y == height-1) ? 0 : y+1;

      Eigen::Vector3f left = (*organized_cloud)(x_left, y).getVector4fMap().head<3>();
      Eigen::Vector3f top  = (*organized_cloud)(x, y_top).getVector4fMap().head<3>();
      Eigen::Vector3f right = (*organized_cloud)(x_right, y).getVector4fMap().head<3>();
      Eigen::Vector3f bottom = (*organized_cloud)(x, y_bottom).getVector4fMap().head<3>();
  
      std::vector<Eigen::Vector3f>normal_calc;
      normal_calc.push_back(left);
      normal_calc.push_back(top);
      normal_calc.push_back(right);
      normal_calc.push_back(bottom);
      normal_calc.push_back(left);
      
      std::vector<Eigen::Vector3f>::iterator it;
      std::vector<Eigen::Vector3f> normals_vec;
      for(it = normal_calc.begin(); it != normal_calc.end()-1; ++it){
        Eigen::Vector3f a = *it;
        Eigen::Vector3f b = *(it+1);
        if(a.allFinite() && b.allFinite()){
          Eigen::Vector3f s = a - self;
          Eigen::Vector3f t = b - self;
          Eigen::Vector3f normal = s.cross(t).normalized();
          normals_vec.push_back(normal);
        }
      }
      if(normals_vec.size() < 1){
        (*normals)(x, y).normal[0] = nan("");
        (*normals)(x, y).normal[1] = nan("");
        (*normals)(x, y).normal[2] = nan("");
      }else{
        std::vector<Eigen::Vector3f>::iterator n_iter;
        Eigen::Vector3f normal = normals_vec.front();
        for(n_iter = normals_vec.begin()+1; n_iter != normals_vec.end(); ++n_iter){
          normal += *n_iter;
        }
        normal.normalize();
        (*normals)(x, y).normal[0] = normal[0];
        (*normals)(x, y).normal[1] = normal[1];
        (*normals)(x, y).normal[2] = normal[2];
        normals_cnt++;
      }
    }
  }
  ROS_INFO("Estimated normals: %d", normals_cnt);

}
void OrganizedPointNormals::spinOnce(){
  
  // handle pcl viewer
  if(show_viewer && viewer){
      viewer->spinOnce();
      if(viewer->wasStopped()){
        viewer->close();
        delete viewer;
        viewer = 0;
    }
  }
}

#include <pcl/common/transforms.h>
int main(int argc, char** args){
  ros::init(argc, args, "organized_point_normals");
  ros::NodeHandle nh;
  OrganizedPointNormals opn(nh);

  while (ros::ok())
  {
    ros::spinOnce();
    opn.spinOnce();
  }

}
