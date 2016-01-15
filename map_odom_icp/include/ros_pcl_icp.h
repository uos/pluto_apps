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
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <map_odom_icp/IcpSrv.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

class RosPclIcp{

  public:
    static const unsigned int ICP_TYPE_POINT_TO_POINT = 0;
    static const unsigned int ICP_TYPE_WITH_NORMALS = 1;

    RosPclIcp(ros::NodeHandle &nh);
    ~RosPclIcp();

    bool registerCloud(
      const sensor_msgs::PointCloud2& cloud,
      const geometry_msgs::PoseStamped& cloud_pose,
      geometry_msgs::PoseStamped &result_pose,
      geometry_msgs::Transform &delta_transform);

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

    bool isTargetCloudSet();

    void setTargetCloud(
      const sensor_msgs::PointCloud2::ConstPtr& target_cloud,
      geometry_msgs::PoseStamped& target_pose);
    
    bool isTargetCloudPrepared();
    void prepareTargetCloud();
    
    void setIcpType(int type);
    void setCorrespondenceDistance(double dist);
    void setTransformationEpsilon(double epsilon);
    void setDownsampleTarget(bool downsample_target);
    void setDownsampleSource(bool downsample_source);
    void setDownsampleLeafSize(double leafzize);
    void setMaximumIterations(int iterations);
    void setMaximumRejectionDistance(double dist);
    void setMaximumRejectionAngle(double angle);

    void showViewer(bool enable);

  
  private:
    ros::NodeHandle nh_;
    ros::ServiceServer service;

   

    template <typename PointType>
      void convertPointCloud2ToPcl(
          const sensor_msgs::PointCloud2& cloud, 
          typename pcl::PointCloud<PointType>& pcl_cloud_xyz){

        // convert point clouds ro pcl
        pcl::PCLPointCloud2 pcl_cloud2;
        pcl_conversions::toPCL(cloud, pcl_cloud2);
        pcl::fromPCLPointCloud2(pcl_cloud2, pcl_cloud_xyz);

      }
    
    bool registerCloudsSrv( map_odom_icp::IcpSrv::Request &req,
                            map_odom_icp::IcpSrv::Response &res);
    template <typename PointTypeA, typename PointTypeB>
      void estimateNormals(
          typename pcl::PointCloud<PointTypeA>::Ptr xyz_in, 
          typename pcl::PointCloud<PointTypeB>::Ptr xyz_normals_out,
          float radius)
      {
        typename pcl::NormalEstimation<PointTypeA, PointTypeB> norm_est;
        typename pcl::search::KdTree<PointTypeA>::Ptr tree (new typename pcl::search::KdTree<PointTypeA> ());
        norm_est.setSearchMethod (tree);
        //norm_est.setKSearch (30);
        norm_est.setRadiusSearch(radius);

        norm_est.setInputCloud (xyz_in);
        norm_est.compute (*xyz_normals_out);
      }

    void tfToEigen( const tf::Transform &transform_tf,
                    Eigen::Matrix4f &transform_eigen);
    
    void eigenToTf( const Eigen::Matrix4f &transform_eigen,
                    tf::Transform &transform_tf);

    template<typename PointType>
    void downsampleCloud(
        typename pcl::PointCloud<PointType>::Ptr &input,
        typename pcl::PointCloud<PointType>::Ptr &output,
        float leafSize)
    {
      typename pcl::VoxelGrid<PointType>sor;
      sor.setInputCloud (input);
      sor.setLeafSize (leafSize, leafSize, leafSize);
      sor.filter(*output);
    }

    void downsampleCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
      float leafSize);

    int m_icp_type;
    sensor_msgs::PointCloud2::ConstPtr m_target_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_target_xyz_ptr;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_target_xyz_normals_ptr;
    geometry_msgs::PoseStamped m_target_pose;
    double m_correspondence_distance;
    bool m_downsample_target;
    bool m_downsample_source;
    double m_downsample_leafsize;
    int m_maximum_iterations;
    double m_transformation_epsilon;
    double m_maximum_rejection_distance;
    double m_maximum_rejection_angle;
    bool m_target_cloud_set;
    bool m_target_cloud_prepared;
    
    void viewerLoop();
    void startViewer();
    pcl::visualization::PCLVisualizer* m_viewer;
    boost::thread m_viewer_thread;
    bool m_viewer_enabled;
    boost::mutex m_viewer_mtx;

};


#endif /* ros_pcl_icp.h */
