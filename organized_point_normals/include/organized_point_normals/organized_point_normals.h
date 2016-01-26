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
 *  organized_point_normals.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ORGANIZED_POINT_NORMALS_H_
#define ORGANIZED_POINT_NORMALS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <organized_point_normals/OrganizedPointNormalsConfig.h>
#include <pcl/visualization/cloud_viewer.h>

class OrganizedPointNormals{
  
  public:
    enum NE_MODE{NE_MODE_ORGANIZED_FAST_NORMALS, NE_MODE_COVARIANCE_MATRIX, NE_MODE_AVERAGE_3D_GRADIENT, NE_MODE_AVERAGE_DEPTH_CHANGE, NE_MODE_SIMPLE_3D_GRADIENT};
    
    OrganizedPointNormals(ros::NodeHandle& nh);
    ~OrganizedPointNormals();
    void spinOnce();
  protected:
    void reconfigureCallback(organized_point_normals::OrganizedPointNormalsConfig& config, uint32_t level);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr);

    void calculateOrganizedFastNormals(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud,
      pcl::PointCloud<pcl::Normal>::Ptr normals);
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    dynamic_reconfigure::Server<organized_point_normals::OrganizedPointNormalsConfig> reconfigure_server;
    double max_depth_change_factor;
    double normal_smoothing_size;
    double radius_search_knn;
    int normal_estimation_mode;
    bool show_viewer;
    pcl::visualization::PCLVisualizer* viewer;

};

#endif /* organized_point_normals.h */
