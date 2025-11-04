#pragma once

#include <Ogre.h>

#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp> 
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include <Ogre.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_rendering/geometry.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/mesh_loader.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include "waypoint_nav_frame.hpp"

#include <boost/thread/mutex.hpp>

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class PanelDockWidget;
} 
 // namespace rviz

namespace waypoint_nav_plugin
{ 
class WaypointNavPanel;
}  // namespace waypoint_nav_plugin 

namespace waypoint_nav_plugin
{

class WaypointNavTool: public rviz_common::Tool, public std::enable_shared_from_this<WaypointNavTool> {

Q_OBJECT
public:
  WaypointNavTool();
  ~WaypointNavTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();
  void timerEvent(QTimerEvent * event) override;

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);
  // virtual int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

  void makeIm(const Ogre::Vector3 &position, const Ogre::Quaternion &quat, int current_mission, bool is_base_mission=false);
  rclcpp::Node::SharedPtr createNewNode(const std::string & node_name);

  int mission_index=1;
  int marker_id_;
  std::string wp_name_str;
  int wp_index;

private:

  std::shared_ptr<std::thread> spin_thread_;
  interactive_markers::MenuHandler menu_handler_;
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  visualization_msgs::msg::InteractiveMarkerControl & makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg);

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::vector<tf2::Vector3> positions_;

  void getMarkerPoses();
  int timer_id_; 
  
  std::string mission_str;

  Ogre::Entity* entity;
  std::string flag_resource_;
  Ogre::SceneNode* moving_flag_node_;
  bool placing_waypoint_;
  bool is_tool_active_;
  Ogre::Vector3 getPointOnPlaneFromWindowXY(const rviz_common::ViewportMouseEvent& event, 
                                                  Ogre::SceneManager* scene_manager, 
                                                  const Ogre::Plane& ground_plane);

  // the waypoint nav Qt frame
  WaypointNavPanel* frame_;
  rviz_common::PanelDockWidget* frame_dock_;

  rclcpp::Node::SharedPtr client_node_;

  
  // map that stores waypoints based on unique names
  std::map<int, Ogre::SceneNode*> waypointNodeMap_;

  // index used for creating unique marker names
};

}  // end namespace kr_rviz_common_plugins