#include "rviz_plugin/waypoint_nav_tool.hpp"

using namespace std::placeholders;

namespace waypoint_nav_plugin
{

WaypointNavTool::WaypointNavTool()
    : moving_flag_node_(NULL)
    , frame_dock_(NULL)
    , frame_(NULL)
    , marker_id_(0) 
    , placing_waypoint_(true) 
    , is_tool_active_(false)
    , client_node_(rclcpp::Node::make_shared("waypoint_navigation_tool"))
{         
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_marker_server",     
                                                                            client_node_->get_node_base_interface(),
                                                                            client_node_->get_node_clock_interface(),
                                                                            client_node_->get_node_logging_interface(),
                                                                            client_node_->get_node_topics_interface(),
                                                                            client_node_->get_node_services_interface());
  RCLCPP_INFO(rclcpp::get_logger("waypoint_tool"), "Construction is complete");
  
  spin_thread_ = std::make_shared<std::thread>([this]() {
      rclcpp::spin(client_node_);
  });
  // timer_id_ = startTimer(100);

  shortcut_key_ = 'w';
}

WaypointNavTool::~WaypointNavTool() {
  if (client_node_ && rclcpp::ok()) {
    rclcpp::shutdown();
  }

  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }

  if (server_) {
    server_->clear();
    server_->applyChanges();
    server_.reset();
  }

  for (auto &entry : waypointNodeMap_) {
    if (entry.second) {
      scene_manager_->destroySceneNode(entry.second);
    }
  }
  waypointNodeMap_.clear();

  delete frame_;
  delete frame_dock_;
}

void WaypointNavTool::onInitialize() {
  flag_resource_ = "package://rviz_plugin/media/waypoint.dae";

  if (rviz_rendering::loadMeshFromResource(flag_resource_).isNull())  {
    RVIZ_COMMON_LOG_ERROR_STREAM("WaypointNavTool: failed to load model resource " << flag_resource_);
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  entity = scene_manager_->createEntity(flag_resource_);
  // Rotate the visual
  auto visual_node = moving_flag_node_->createChildSceneNode(); 
  visual_node->attachObject(entity);
  visual_node->setOrientation(Ogre::Quaternion(Ogre::Degree(-91.8), Ogre::Vector3::UNIT_Z));
  moving_flag_node_->setVisible(false);

  rviz_common::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new WaypointNavPanel(nullptr, context_, &waypointNodeMap_, server_, std::make_shared<int>(marker_id_), this);


  if (window_context) {
    frame_dock_ = window_context->addPane("Waypoint Navigation", frame_);
  }

  frame_->enable();

  RCLCPP_INFO(rclcpp::get_logger("waypoint_tool"), "Initialization success");
}

void WaypointNavTool::activate() {
  moving_flag_node_->setVisible(false);
  is_tool_active_ = true;

  RCLCPP_INFO(rclcpp::get_logger("waypoint_tool"), "Activation complete");
}


void WaypointNavTool::deactivate() {
  if (moving_flag_node_) {
    moving_flag_node_->setVisible(false);
  }
  placing_waypoint_ = true;
  is_tool_active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("waypoint_tool"), "Deactivation complete. States reset.");
}

// Handling mouse events
int WaypointNavTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
  if (!moving_flag_node_ || is_tool_active_ == false) {
    return Render;
  }

  auto projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  auto projection = projection_finder->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);
  Ogre::Vector3 intersection = projection.second;

  if (projection.first) {
    if (placing_waypoint_) {
      // First click: Place Waypoint location
      if (event.leftDown()) {
        moving_flag_node_->setVisible(true);
        if (intersection.x == 0 && intersection.y == 0) return Render;
        moving_flag_node_->setPosition(intersection);
        placing_waypoint_ = false;
        RCLCPP_INFO(rclcpp::get_logger("waypoint_tool"), "Waypoint location set. Adjust heading and click again.");
      }
    } else {
      // Adjust heading based on cursor position
      Ogre::Vector3 direction = intersection - moving_flag_node_->getPosition();
      direction.z = 0; // Keep heading on the XY plane
      if (!direction.isZeroLength()) {
        direction.normalise();
        Ogre::Quaternion quat = Ogre::Vector3::UNIT_X.getRotationTo(direction);
        moving_flag_node_->setOrientation(quat);
      }

      // Second click: Finalize waypoint
      if (event.leftDown()) {
        frame_->insertWaypoint(moving_flag_node_->getPosition(), moving_flag_node_->getOrientation());
        placing_waypoint_ = true;
        moving_flag_node_->setVisible(false);
        RCLCPP_INFO(rclcpp::get_logger("waypoint_tool"), "Waypoint finalized.");
        return Render | Finished;
      }
    }
  }

  return Render;
}

void WaypointNavTool::makeIm(const Ogre::Vector3 &position, const Ogre::Quaternion &quat, int current_mission, bool is_base_mission) {
  marker_id_++; // increment the index for unique marker names

  std::stringstream wp_name, wp_desc;
  wp_name << wp_name_prefix << marker_id_;
  wp_name_str = wp_name.str();
  // std::cout << "wp name: " << wp_name_str << std::endl;

  mission_str = std::to_string((current_mission+1)/2);
  mission_str += current_mission%2? "_pre" : "_in"; 
  wp_desc << wp_name_prefix << marker_id_ << " " << mission_str;
  std::string wp_desc_str(wp_desc.str());

  if (rviz_rendering::loadMeshFromResource(flag_resource_).isNull()) {
    RCLCPP_ERROR(rclcpp::get_logger("WaypointNavTool"), "Falied to load model from %s", flag_resource_.c_str());
    return;
  }

  // create a new flag in the Ogre scene and save it in a std::map.
  Ogre::SceneNode *sn_ptr = scene_manager_->getRootSceneNode()->createChildSceneNode();
  entity = scene_manager_->createEntity(flag_resource_);
  auto sn_ptr_visual = sn_ptr->createChildSceneNode();
  sn_ptr_visual->attachObject(entity);
  sn_ptr_visual->setOrientation(Ogre::Quaternion(Ogre::Degree(-93.8), Ogre::Vector3::UNIT_Z)); // Rotate only the visual
  sn_ptr->setVisible(true);
  sn_ptr->setPosition(position);
  sn_ptr->setOrientation(quat);

  std::map<int, Ogre::SceneNode*>::iterator sn_entry = waypointNodeMap_.find(marker_id_);

  if (sn_entry == waypointNodeMap_.end()) {
    waypointNodeMap_.insert(std::make_pair(marker_id_, sn_ptr));
    waypointNodeMap_.insert(std::make_pair(marker_id_ + 1000, sn_ptr_visual)); // Pair the sn_ptr_visual with a unique key
  } else {
    RCLCPP_WARN(rclcpp::get_logger("waypoint_nav_tool"), "%s already in map", wp_name_str.c_str());
    return;
  }

  auto pos = geometry_msgs::msg::PoseStamped();
  pos.pose.position.x = position.x;
  pos.pose.position.y = position.y;
  pos.pose.position.z = position.z;
  pos.pose.orientation.x = quat.x;
  pos.pose.orientation.y = quat.y;
  pos.pose.orientation.z = quat.z;
  pos.pose.orientation.w = quat.w;

  auto int_marker = visualization_msgs::msg::InteractiveMarker();
  int_marker.header.stamp = rclcpp::Clock().now();
  int_marker.header.frame_id = frame_->getFrameId().toStdString();
  int_marker.pose = pos.pose;
  int_marker.scale = 1.5;
  int_marker.name = wp_name_str;
  int_marker.description = "";

  // create a cylinder marker
  auto cyn_marker = visualization_msgs::msg::Marker();
  cyn_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  cyn_marker.scale.x = 1.0;
  cyn_marker.scale.y = 1.0;
  cyn_marker.scale.z = 0.0;

  float color[3];
  if (current_mission % 2 ==0) {
    color[0] = 0; 
    color[1] = 1; 
    color[2] = 0;
  } else {
    color[0] = 1; 
    color[1] = 1; 
    color[2] = 0;
  }

  cyn_marker.color.r = color[0];
  cyn_marker.color.g = color[1];
  cyn_marker.color.b = color[2];
  cyn_marker.color.a = 0.5;

  // create a text marker for the waypoint label
  auto text_marker = visualization_msgs::msg::Marker();
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.text = wp_desc_str;  // Display waypoint name and mission
  text_marker.scale.z = 0.5;  // Text height (bigger for better visibility)
  text_marker.color.r = 0.0;  // Black color
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;  // Fully opaque
  text_marker.pose.position.z = 1.0;  // Position text above the waypoint

  // create a non-interactive control which contains the markers
  auto cyn_control = visualization_msgs::msg::InteractiveMarkerControl();
  cyn_control.always_visible = true;
  cyn_control.markers.push_back(cyn_marker);
  cyn_control.markers.push_back(text_marker);  // Add text marker

  // add the control to the interactive marker
  int_marker.controls.push_back(cyn_control);

  auto control = visualization_msgs::msg::InteractiveMarkerControl();
  control.orientation.w = 0.707106781;
  control.orientation.x = 0;
  control.orientation.y = 0.707106781;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  // control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; // up and down
  // int_marker.controls.push_back(control);
  // control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  // int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_delete";
  int_marker.controls.push_back(control);

  // std::cout << "mission: " << current_mission << std::endl;
  if (!is_base_mission) {
    server_->insert(int_marker);
    server_->setCallback(int_marker.name, std::bind(&WaypointNavTool::processFeedback, this, std::placeholders::_1));
    server_->applyChanges();
    // menu_handler_.apply(*server_, int_marker.name);

    //Set the current marker as selected
    wp_index = std::stoi(wp_name_str.substr(strlen(wp_name_prefix), 2));
    Ogre::Vector3 p = position;
    Ogre::Quaternion q = quat;
    frame_->setSelectedMarkerName(wp_name_str);
    frame_->setWpLabel();
    frame_->setPose(p, q, wp_index);
  }
}

void WaypointNavTool::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)  {
  switch (feedback->event_type) {
  case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT: {
    std::map<int, Ogre::SceneNode*>::iterator sn_entry =
        waypointNodeMap_.find(std::stoi(feedback->marker_name.substr(strlen(wp_name_prefix))));
    if (sn_entry == waypointNodeMap_.end())
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_tool"), "%s not found in map", feedback->marker_name.c_str());
    else {
      if (feedback->menu_entry_id == 1) {
        // Delete selected waypoint
        std::stringstream wp_name;
        wp_name << wp_name_prefix << sn_entry->first;
        wp_name_str = wp_name.str();
        server_->erase(wp_name_str);

        // menu_handler_.reApply(*server_);
        server_->applyChanges();
        sn_entry->second->detachAllObjects();
        waypointNodeMap_.erase(sn_entry);
      } else {
        // Set the pose manually from the line edits
        Ogre::Vector3 position;
        Ogre::Quaternion quat;

        frame_->getPose(position, quat);

        auto pos = geometry_msgs::msg::Pose();
        pos.position.x = position.x;
        pos.position.y = position.y;
        pos.position.z = position.z;

        pos.orientation.x = quat.x;
        pos.orientation.y = quat.y;
        pos.orientation.z = quat.z;
        pos.orientation.w = quat.w;

        sn_entry->second->setPosition(position);
        sn_entry->second->setOrientation(quat);

        // Also update the visual child node to ensure it stays synchronized
        std::map<int, Ogre::SceneNode*>::iterator visual_entry = waypointNodeMap_.find(sn_entry->first + 1000);
        if (visual_entry != waypointNodeMap_.end()) {
          // Ensure the visual child maintains its offset rotation
          visual_entry->second->setOrientation(Ogre::Quaternion(Ogre::Degree(-93.8), Ogre::Vector3::UNIT_Z));
        }

        frame_->setWpLabel();

        server_->setPose(feedback->marker_name, pos);
        server_->applyChanges();
      }
    }
  }
  break;
  case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE: {
    std::map<int, Ogre::SceneNode*>::iterator sn_entry = waypointNodeMap_.find(std::stoi(feedback->marker_name.substr(strlen(wp_name_prefix))));
    wp_index = std::stoi(feedback->marker_name.substr(strlen(wp_name_prefix))); // index map = index wp
    // std::cout << "map index: " << map_index << std::endl;
    if (sn_entry == waypointNodeMap_.end())
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_tool"), "%s not found in map", feedback->marker_name.c_str());
    else {
      auto pos = geometry_msgs::msg::PoseStamped();
      pos.pose = feedback->pose;

      Ogre::Vector3 position;
      position.x = pos.pose.position.x;
      position.y = pos.pose.position.y;
      position.z = pos.pose.position.z;

      Ogre::Quaternion quat;
      quat.x = pos.pose.orientation.x;
      quat.y = pos.pose.orientation.y;
      quat.z = pos.pose.orientation.z;
      quat.w = pos.pose.orientation.w;

      // Update parent node (this contains the arrow visual as a child)
      sn_entry->second->setPosition(position);
      sn_entry->second->setOrientation(quat);

      // Also update the visual child node to ensure it stays synchronized
      // The visual node is stored at marker_id + 1000
      std::map<int, Ogre::SceneNode*>::iterator visual_entry = waypointNodeMap_.find(sn_entry->first + 1000);
      if (visual_entry != waypointNodeMap_.end()) {
        // Ensure the visual child maintains its offset rotation (-93.8 degrees on Z axis)
        visual_entry->second->setOrientation(Ogre::Quaternion(Ogre::Degree(-93.8), Ogre::Vector3::UNIT_Z));
      }

      frame_->setWpLabel();
      frame_->setPose(position, quat, wp_index);
      frame_->setSelectedMarkerName(feedback->marker_name);
    }
  }
  break;
  }
}

void WaypointNavTool::getMarkerPoses() {
  std::map<int, Ogre::SceneNode*>::iterator sn_it;
  for (sn_it = waypointNodeMap_.begin(); sn_it != waypointNodeMap_.end(); sn_it++) {
    auto int_marker = visualization_msgs::msg::InteractiveMarker();

    std::stringstream wp_name;
    wp_name << wp_name_prefix << sn_it->first;
    wp_name_str = wp_name.str();
    server_->get(wp_name_str, int_marker);

    RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_tool"), "position: x: %f, y: %f, z: %f", 
                                                          int_marker.pose.position.x,
                                                          int_marker.pose.position.y,
                                                          int_marker.pose.position.z);
  }
}

rclcpp::Node::SharedPtr WaypointNavTool::createNewNode(const std::string & node_name) {
  std::string node = "__node:=" + node_name;
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node, "--"});
  return std::make_shared<rclcpp::Node>("_", options);
}

void WaypointNavTool::timerEvent(QTimerEvent * event) {
  if (event->timerId() == timer_id_) rclcpp::spin_some(client_node_);
}
} // end namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_nav_plugin::WaypointNavTool, rviz_common::Tool)