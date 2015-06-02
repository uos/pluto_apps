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

#include <costmap_2d/footprint_layer.h>



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

      unsigned int dx_;
      unsigned int dy_;

      double diff_range_x_;
      double diff_range_y_;
      double diff_range_z_;
      double diff_max_z_;

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
