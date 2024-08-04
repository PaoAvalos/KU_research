// File: custom_costmap_layer/src/custom_layer.cpp

#include <custom_costmap_layer/custom_layer.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_math.h>

PLUGINLIB_EXPORT_CLASS(custom_costmap_layer::CustomLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace custom_costmap_layer
{

CustomLayer::CustomLayer() : marker_received_(false), enabled_(true), min_x_(0.0), min_y_(0.0), max_x_(0.0), max_y_(0.0) {
  ROS_INFO("CustomLayer constructor called");

}

void CustomLayer::onInitialize()
{
  ROS_INFO("CustomLayer::onInitialize() called");
  ros::NodeHandle nh("~/" + name_);
  dsrv_ = new dynamic_reconfigure::Server<CustomLayerConfig>(nh);
  dynamic_reconfigure::Server<CustomLayerConfig>::CallbackType cb = boost::bind(&CustomLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  marker_sub_ = nh.subscribe("/visualization_marker", 10, &CustomLayer::markerCallback, this);
  ROS_INFO("CustomLayer subscribed to /visualization_marker");

}

void CustomLayer::reconfigureCB(CustomLayerConfig &config, uint32_t level)
{
  ROS_INFO("CustomLayer::reconfigureCB() called");
  enabled_ = config.enabled;
}

void CustomLayer::markerCallback(const visualization_msgs::Marker& msg)
{
  ROS_INFO("CustomLayer::markerCallback() called");
  marker_ = msg;
  marker_received_ = true;
  updateBounds(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z, &min_x_, &min_y_, &max_x_, &max_y_);
}

void CustomLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !marker_received_)
    return;

  double marker_x = marker_.pose.position.x;
  double marker_y = marker_.pose.position.y;
  double marker_size_x = marker_.scale.x / 2.0;
  double marker_size_y = marker_.scale.y / 2.0;

  *min_x = std::min(*min_x, marker_x - marker_size_x);
  *min_y = std::min(*min_y, marker_y - marker_size_y);
  *max_x = std::max(*max_x, marker_x + marker_size_x);
  *max_y = std::max(*max_y, marker_y + marker_size_y);
}

void CustomLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !marker_received_)
    return;

  double marker_x = marker_.pose.position.x;
  double marker_y = marker_.pose.position.y;
  double marker_size_x = marker_.scale.x;
  double marker_size_y = marker_.scale.y;

  unsigned int mx, my;
  if (master_grid.worldToMap(marker_x, marker_y, mx, my))
  {
    // Ensure marker sizes are integers or properly cast
    int marker_size_x_cells = static_cast<int>(marker_size_x);
    int marker_size_y_cells = static_cast<int>(marker_size_y);

    for (int i = -marker_size_x_cells / 2; i < marker_size_x_cells / 2; ++i)
    {
      for (int j = -marker_size_y_cells / 2; j < marker_size_y_cells / 2; ++j)
      {
        int x = mx + i;
        int y = my + j;
        if (x >= 0 && x < master_grid.getSizeInCellsX() && y >= 0 && y < master_grid.getSizeInCellsY())
        {
          master_grid.setCost(x, y, LETHAL_OBSTACLE);
        }
      }
    }
  }
}

} // end namespace custom_costmap_layer
