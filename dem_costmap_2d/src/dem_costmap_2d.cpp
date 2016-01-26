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
 *  dem_costmap_2d.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include <dem_costmap_2d/dem_costmap_2d.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dem_costmap_2d::DEMCostmap2D, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;


namespace dem_costmap_2d
{

void DEMCostmap2D::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  ros::NodeHandle nh_g;
  current_ = true;
  default_value_ = FREE_SPACE;
  DEMCostmap2D::matchSize();
  setupDynamicReconfigure(nh);
  sub = nh_g.subscribe("octomap_full", 10, &DEMCostmap2D::octomapCallback, this);
  ROS_INFO("DEM Costmap 2D Plugin loaded as %s", name_.c_str());
  
  nh.param("diff_range_x", diff_range_x_, 0.1);
  nh.param("diff_range_y", diff_range_y_, 0.1);
  nh.param("diff_range_z", diff_range_z_, 0.6);
  nh.param("diff_max_z", diff_max_z_, 0.2);
  nh.param("z_max", z_max_, 2.3);
}

void DEMCostmap2D::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg){
  ROS_INFO("Received OcTree");
  
  octomap::AbstractOcTree* abstract_tree = fullMsgToMap(*msg);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);

  Costmap2D::resetMaps();
  resizeMap(octree);
  buildHeightDifferenceMap(octree);
}

void DEMCostmap2D::resizeMap(octomap::OcTree* octree){
  double size_x, size_y, size_z;
  double min_x, min_y, min_z;

  octree->getMetricSize(size_x, size_y, size_z);
  octree->getMetricMin(min_x, min_y, min_z);
  unsigned int dx = (int) (size_x / octree->getResolution() + 0.5);
  unsigned int dy = (int) (size_y / octree->getResolution() + 0.5);
  
  ROS_INFO("new map size in meters: (%f, %f)", size_x, size_y);
  
  double resolution = octree->getResolution();
  double origin_x = min_x;
  double origin_y = min_y;

  layered_costmap_->resizeMap(dx, dy, resolution, origin_x, origin_y);
}

void DEMCostmap2D::buildHeightDifferenceMap(octomap::OcTree* octree){
  
  std::map<unsigned int, octomap::OcTree::leaf_iterator>height_map; 

  octomap::OcTree::leaf_iterator end;
  octomap::OcTree::leaf_iterator it;

  // build height map with in the tree lowest cells
  for(it = octree->begin_leafs(), end = octree->end_leafs();
    it != octree->end_leafs(); ++it)
  {
    if(octree->isNodeOccupied(*it)){
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      
      if(z > z_max_){
        continue;
      }

      unsigned int mx, my;
      worldToMap(x, y, mx, my);

      unsigned int index = getIndex(mx, my);
      std::map<unsigned int, octomap::OcTree::leaf_iterator>::iterator
        elem = height_map.find(index); 
      if(elem != height_map.end()){
        if(z < elem->second.getZ()){
          height_map[index] = it;
        }
      }
      else{
        height_map[index] = it;
      }
    }
  }

  hdm_mtx.lock();
  // clear the map
  height_diff_map_.clear();

  // calculate height differences
  std::map<unsigned int, octomap::OcTree::leaf_iterator>::iterator hm_iter, hm_end;
  for(hm_iter = height_map.begin(), hm_end = height_map.end();
    hm_iter != hm_end; ++hm_iter)
  {
    octomap::OcTree::leaf_iterator center_it = hm_iter->second;
    octomath::Vector3 center = center_it.getCoordinate();
    octomath::Vector3 diff(diff_range_x_, diff_range_y_, diff_range_z_);
    octomath::Vector3 min_bbx = center - diff;
    octomath::Vector3 max_bbx = center + diff;

    octomap::OcTree::leaf_bbx_iterator bbx_iter, bbx_end;

    unsigned int mx, my;
    worldToMap(center.x(), center.y(), mx, my);
    unsigned int index = getIndex(mx, my);
    height_diff_map_[index] = 0;
    for(bbx_iter = octree->begin_leafs_bbx(min_bbx, max_bbx),
      bbx_end = octree->end_leafs_bbx();
      bbx_iter != bbx_end; ++bbx_iter)
    {
      if(octree->isNodeOccupied(*bbx_iter)){
        double diff = std::abs(bbx_iter.getZ() - center.z());
        if(height_diff_map_[index] < diff){
          height_diff_map_[index] = diff;
        }
      }
    }
  }
  hdm_mtx.unlock();
}

unsigned int DEMCostmap2D::diffToMapCosts(double diff){
  double factor = diff / diff_max_z_;
  if(factor > 1){
    return LETHAL_OBSTACLE;
  }else{
    return (unsigned int) ((CIRCUMSCRIBED - 1) * factor + 0.5);
  }
}


void DEMCostmap2D::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<dem_costmap_2d::DEMCostmap2DConfig>(nh);
  dynamic_reconfigure::Server<dem_costmap_2d::DEMCostmap2DConfig>::CallbackType cb = boost::bind(
      &DEMCostmap2D::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

DEMCostmap2D::~DEMCostmap2D()
{
  if(dsrv_)
    delete dsrv_;
}

void DEMCostmap2D::reconfigureCB(dem_costmap_2d::DEMCostmap2DConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  diff_range_x_ = config.diff_range_x;
  diff_range_y_ = config.diff_range_y;
  diff_range_z_ = config.diff_range_z;
  diff_max_z_ = config.diff_max_z;
  z_max_ = config.z_max;
}

void DEMCostmap2D::updateBounds(
  double robot_x,
  double robot_y,
  double robot_yaw,
  double* min_x,
  double* min_y,
  double* max_x,
  double* max_y
){
  if (!enabled_)
    return;

  useExtraBounds(min_x, min_y, max_x, max_y);

  current_ = true;

  unsigned int mx;
  unsigned int my;

  std::map<unsigned int, double>::iterator dem_iter, dem_end;
  hdm_mtx.lock();
  for(dem_iter = height_diff_map_.begin(), dem_end = height_diff_map_.end();
    dem_iter != dem_end; ++dem_iter){
    
    unsigned int index = dem_iter->first;
    unsigned int cost = diffToMapCosts(dem_iter->second);
    
    int my = index / size_x_;
    int mx = index % size_x_;
    
    double wx, wy;
    mapToWorld(mx, my, wx, wy);
    touch(wx, wy, min_x, min_y, max_x, max_y);
    
    setCost(mx, my, cost);
  }
  hdm_mtx.unlock();

}

void DEMCostmap2D::updateCosts(
  costmap_2d::Costmap2D& master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j
){
  if (!enabled_)
    return;

  int combination_method_ = 0;

  switch(combination_method_){
    case 0: // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1: // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default: // Nothing
      break;
  }
}

}  // namespace dem_costmap_2d_layer
