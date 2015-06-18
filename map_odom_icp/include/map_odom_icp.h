#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros_pcl_icp.h>
#include <stack>
#include <boost/thread/mutex.hpp>

class MapOdomICP{

  public:
    MapOdomICP(ros::NodeHandle &nh);
    void sendMapToOdomCombined();
  private:
    RosPclIcp icp_;

    class Sync{
      public:
        static void setUpdated();
        static bool hasUpdated();
      private:
        static boost::mutex mtx_; 
        static bool updated_;
    };

    boost::mutex target_mtx_;

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher cloud_pub_;
    
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    //std::stack<tf::StampedTransform>map_to_odom_;
    //std::stack<sensor_msgs::PointCloud2> clouds_;
    //std::stack<geometry_msgs::PoseStamped> poses_;

    sensor_msgs::PointCloud2::ConstPtr target_cloud_;
    sensor_msgs::PointCloud2::ConstPtr previous_cloud_;

    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped previous_pose_;
    tf::StampedTransform map_to_odom_;

    void storeTargetCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud);
    void pointCloud2Callback(const::sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool icpWithTargetCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool icpWithPreviousCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud);

    bool icpUpdateMapToOdomCombined(
      const sensor_msgs::PointCloud2::ConstPtr &target,
      const geometry_msgs::PoseStamped &target_pose,
      const sensor_msgs::PointCloud2::ConstPtr &cloud,
      geometry_msgs::PoseStamped &result_pose
    );
    
    bool getPointCloudPose(
      const sensor_msgs::PointCloud2 &cloud,
      const std::string &fixed_frame,
      geometry_msgs::PoseStamped &pose);

    // ros_pcl parameters
    double correspondence_distance_;
    double transformation_epsilon_;
    int maximum_iterations_;
    bool downsample_target_;
    bool downsample_cloud_;
    double downsample_leafsize_;
    
    bool publish_cloud_;
    bool use_target_cloud_;

};
