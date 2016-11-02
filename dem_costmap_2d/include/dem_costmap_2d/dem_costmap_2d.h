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
 *  dem_costmap_2d.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */



#ifndef DEM_COSTMAP_2D_H_
#define DEM_COSTMAP_2D_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dem_costmap_2d/DEMCostmap2DConfig.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <boost/thread/mutex.hpp>



namespace dem_costmap_2d
{

  class DEMCostmap2D : public costmap_2d::CostmapLayer
  {
    public:
      DEMCostmap2D()
      {
        costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
      }

      virtual ~DEMCostmap2D();
      virtual void onInitialize();

      virtual void updateBounds(
          double robot_x,
          double robot_y,
          double robot_yaw,
          double* min_x,
          double* min_y,
          double* max_x,
          double* max_y
          );

      virtual void updateCosts(
          costmap_2d::Costmap2D& master_grid,
          int min_i,
          int min_j,
          int max_i,
          int max_j
          );

    protected:
      static const unsigned char CIRCUMSCRIBED = 127;
      virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
      void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
    private:

      boost::mutex hdm_mtx;

      unsigned int dx_;
      unsigned int dy_;

      double diff_range_x_;
      double diff_range_y_;
      double diff_range_z_;
      double diff_max_z_;
      double z_max_;

      unsigned int diffToMapCosts(double diff);
      std::map<unsigned int, double>height_diff_map_;
      void resizeMap(octomap::OcTree* octree);
      void buildHeightDifferenceMap(octomap::OcTree* octree);
      ros::Subscriber sub;
      void reconfigureCB(dem_costmap_2d::DEMCostmap2DConfig &config, uint32_t level);
      dynamic_reconfigure::Server<dem_costmap_2d::DEMCostmap2DConfig> *dsrv_;
  };

}  // namespace dem_costmap_2d

#endif  // DEM_COSTMAP_2D_H_
