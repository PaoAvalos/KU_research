#ifndef CUSTOM_LAYER_H_
#define CUSTOM_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <custom_costmap_layer/CustomLayerConfig.h>

namespace custom_costmap_layer
{

class CustomLayer : public costmap_2d::Layer
{
public:
  CustomLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(CustomLayerConfig &config, uint32_t level);
  void markerCallback(const visualization_msgs::Marker& msg);

  dynamic_reconfigure::Server<CustomLayerConfig> *dsrv_;
  ros::Subscriber marker_sub_;
  visualization_msgs::Marker marker_;
  bool marker_received_;

  bool enabled_;  // Added declaration
  double min_x_;  // Added declaration
  double min_y_;  // Added declaration
  double max_x_;  // Added declaration
  double max_y_;  // Added declaration
};

} // end namespace custom_costmap_layer

#endif // CUSTOM_LAYER_H_
