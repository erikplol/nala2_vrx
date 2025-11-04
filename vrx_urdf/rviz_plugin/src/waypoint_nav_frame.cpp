#include "rviz_plugin/waypoint_nav_frame.hpp"
#include <QMessageBox>
#include <QDir>
#include <QDateTime>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace waypoint_nav_plugin 
{

WaypointNavPanel::WaypointNavPanel(QWidget *parent
                                  , rviz_common::DisplayContext* context
                                  , std::map<int, Ogre::SceneNode* >* waypointNodeMap
                                  , std::shared_ptr<interactive_markers::InteractiveMarkerServer> server
                                  , std::shared_ptr<int> uniqueWaypointIndex
                                  , WaypointNavTool* wp_tool
                                  )
  : rviz_common::Panel(parent)
  , context_(context)
  , server_(server)
  , waypointNodeMap_(waypointNodeMap)
  , uniqueWaypointIndex_(uniqueWaypointIndex)
  , wp_nav_tool_(wp_tool)
  , frame_id_("map")
  , current_mission(0)
  , default_height_(0.0)
  , ui_(std::make_shared<Ui::WaypointNavigationWidget>())
  , selected_marker_name_(std::string(wp_name_prefix) + "1")
  , client_node_(createNewNode("waypoint_navigation_panel"))
{
  if (!context_) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Context is null!");
    return;
  }
  scene_manager_ = context_->getSceneManager();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(client_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // set up the GUI
  if (ui_) {
    ui_->setupUi(this);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "UI pointer is null!");
    return;
  }

  // set up all publisher & client
  try {
    initPublisher();
    initService();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error initializing publishers and services: %s", e.what());
  }

  //connect the Qt signals and slots
  connect(ui_->publish_button, &QPushButton::clicked, this, &WaypointNavPanel::publishWaypoint);
  connect(ui_->calibrate_button, &QPushButton::clicked, this, &WaypointNavPanel::calibratePose);  
  connect(ui_->publish_all_button, &QPushButton::clicked, this, &WaypointNavPanel::publishAllWaypoints);
  connect(ui_->missionComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WaypointNavPanel::setMission);

  connect(ui_->delete_all_button, &QPushButton::clicked, std::bind(&WaypointNavPanel::deleteAllWaypoints, this, true));
  connect(ui_->delete_button, &QPushButton::clicked, this, &WaypointNavPanel::deleteSelectedWaypoint);

  connect(ui_->save_wp_button, &QPushButton::clicked, this, &WaypointNavPanel::saveWaypoint);
  connect(ui_->load_wp_button, &QPushButton::clicked, this, &WaypointNavPanel::loadWaypoint);

  connect(ui_->command_button, &QPushButton::clicked, this, &WaypointNavPanel::autonomy);
  connect(ui_->start_route_button, &QPushButton::clicked, this, &WaypointNavPanel::recordRoute);
  connect(ui_->clear_route_button, &QPushButton::clicked, this, &WaypointNavPanel::clearRoute);
  connect(ui_->save_route_button, &QPushButton::clicked, this, &WaypointNavPanel::saveRoute);
  connect(ui_->load_route_button, &QPushButton::clicked, this, &WaypointNavPanel::loadRoute);

  connect(ui_->x_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);
  connect(ui_->y_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);
  connect(ui_->z_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);
  connect(ui_->yaw_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);

  setMission(0);

  try {
    spin_thread_ = std::make_shared<std::thread>([this]() {
      rclcpp::spin(client_node_);
    });
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error creating spin thread: %s", e.what());
  }

  // timer_id_ = startTimer(100);

  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Waypoint panel construction is complete");
}

WaypointNavPanel::~WaypointNavPanel() {
  try {
    // Stop the spinning thread if it exists
    if (spin_thread_ && spin_thread_->joinable()) {
      // Signal the node to shutdown
      if (client_node_) {
        rclcpp::shutdown();
      }
      spin_thread_->join();
    }
    
    waypointNodeMap_ = NULL;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in destructor: %s", e.what());
  }
}

void WaypointNavPanel::enable() {
  show();
}

void WaypointNavPanel::disable() {
  hide();
}

void WaypointNavPanel::initPublisher() {
  try {
    if (!client_node_) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Client node is null, cannot create publishers");
      return;
    }
    
    wp_pub_[0] = client_node_->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);
    for(int i=0; i<20; i++){
      std::string current_topic =  "/waypoints";
      current_topic += "/mission_" + std::to_string(i/2+1);

      i%2 == 0? current_topic += "/pre" : current_topic += "/in";

      wp_pub_[i+1] = client_node_->create_publisher<nav_msgs::msg::Path>(current_topic, 10);
      RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Created topic %s", current_topic.c_str());
    }

    pose_pub_ = client_node_->create_publisher<nav_msgs::msg::Path>("/rviz/nala_path", 10);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error initializing publishers: %s", e.what());
  }
}

void WaypointNavPanel::initService() {
  try {
    if (!client_node_) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Client node is null, cannot create service clients");
      return;
    }

    path_client_[0] = client_node_->create_client<nala2_interfaces::srv::SetPath>("/waypoints");
    for(int i=0; i<20; i++){
      std::string current_topic =  "/waypoints";
      current_topic += "/mission_" + std::to_string(i/2+1);

      i%2 == 0? current_topic += "/pre" : current_topic += "/in";

      path_client_[i+1] = client_node_->create_client<nala2_interfaces::srv::SetPath>(current_topic);
      RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Created topic %s", current_topic.c_str());
    }

    autonomy_client_ = client_node_->create_client<nala2_interfaces::srv::StringService>("/decisioning/mission_manager/command");
    route_client_ = client_node_->create_client<nala2_interfaces::srv::StringService>("/decisioning/path_record/cmd");
    calibrate_client_ = client_node_->create_client<std_srvs::srv::Trigger>("/perception/localization/localization_node/calibration");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error initializing service clients: %s", e.what());
  }
}

void WaypointNavPanel::setPath() {
  try {
    paths[current_mission].poses.clear();
    std::map<int, Ogre::SceneNode* >::iterator sn_it;
    for (sn_it = waypointNodeMap_->begin(); sn_it != waypointNodeMap_->end(); sn_it++) {
      geometry_msgs::msg::PoseStamped pos;
      Ogre::Vector3 position;
      position = sn_it->second->getPosition();
      // This is quite dangerous, so future nala please fix this for me -mundi :)
      if (position.x == 0.0 && position.y == 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Waypoint with position (0, 0) ignored");
        continue;
      }
      pos.pose.position.x = position.x;
      pos.pose.position.y = position.y;
      pos.pose.position.z = position.z;

      Ogre::Quaternion quat;
      quat = sn_it->second->getOrientation();
      pos.pose.orientation.x = quat.x;
      pos.pose.orientation.y = quat.y;
      pos.pose.orientation.z = quat.z;
      pos.pose.orientation.w = quat.w;

      pos.header.frame_id = "map";
      pos.header.stamp = client_node_->now();

      paths[current_mission].poses.push_back(pos);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in setPath function: %s", e.what());
  }
}

void WaypointNavPanel::drawPath(int mission_index, bool is_base_mission){
  try {
    mission_str = std::to_string((mission_index+1)/2);
    mission_str += (mission_index%2 ? " pre" : " in");
    
    // Don't reset marker_id_ here - it should be reset once before drawing all paths
    // This allows continuous marker IDs across missions when drawing multiple missions
    
    // Draw waypoints from the stored path data
    for(int i = 0; i < paths[mission_index].poses.size(); i++) {
      geometry_msgs::msg::PoseStamped pos = paths[mission_index].poses[i];
      Ogre::Vector3 position;
      position.x = pos.pose.position.x;
      position.y = pos.pose.position.y;
      position.z = pos.pose.position.z;

      Ogre::Quaternion quat = Ogre::Quaternion(Ogre::Radian(M_PI / 2.), Ogre::Vector3::UNIT_Z);
      quat.x = pos.pose.orientation.x;
      quat.y = pos.pose.orientation.y;
      quat.z = pos.pose.orientation.z;
      quat.w = pos.pose.orientation.w;

      wp_nav_tool_->makeIm(position, quat, mission_index, is_base_mission);
    }

    // Only update the path from visual markers if we're editing the current mission
    // Don't update when drawing for base mission view
    if (!is_base_mission && mission_index == current_mission) {
      setPath();
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("waypoint_panel"), 
                 "Finished drawing %zu waypoints for mission %s (base_mission: %s)", 
                 paths[mission_index].poses.size(), 
                 mission_str.c_str(),
                 is_base_mission ? "yes" : "no");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in drawPath function: %s", e.what());
  }
}

void WaypointNavPanel::setMission(int index) {
  try {
    current_mission = index;
    wp_nav_tool_->mission_index = index;
    
    // Clear all visual markers before switching missions
    deleteAllWaypoints(false);  // false = don't clear the paths data

    // Reset marker_id to 0 before drawing any paths
    wp_nav_tool_->marker_id_ = 0;

    if(current_mission == 0){
      // When in "Select Mission" mode, draw all missions from stored paths
      for(int i = 1; i < 21; i++){
        drawPath(i, true);
      }
      ui_->current_topic_label->setText("/waypoints");
      RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Currently in base mission (Select Mission)");
    } else {  
      // Draw only the selected mission
      drawPath(current_mission);
      current_topic =  "/waypoints/mission_";
      current_topic += std::to_string((current_mission+1)/2);
      current_mission%2 == 0? current_topic += "/in" : current_topic += "/pre";
      ui_->current_topic_label->setText(current_topic.c_str());

      mission_str = std::to_string((current_mission+1)/2);
      current_mission%2 == 0? mission_str += " in" : mission_str += " pre";
      RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Currently in mission %s", mission_str.c_str());
    }
    ui_->insert_wp_spinBox->setMinimum(1);
    ui_->insert_wp_spinBox->setMaximum(paths[current_mission].poses.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in setMission function: %s", e.what());
  }
}

void WaypointNavPanel::calibratePose(bool button_clicked) {
  try {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = calibrate_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node_, result, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Calibration successful: %s", result.get()->message.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Failed to call calibration service");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in calibratePose function: %s", e.what());
  }
}

void WaypointNavPanel::deleteAllWaypoints(bool button_clicked) {
  try {
    if (button_clicked) {
      if (current_mission == 0) {
        // In "Select Mission" mode, delete all waypoints from all missions
        for (int i = 1; i < 21; i++) {
          paths[i].poses.clear();
        }
        RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Delete all waypoints clicked - Cleared all missions");
      } else {
        // In specific mission mode, delete only current mission's waypoints
        paths[current_mission].poses.clear();
        RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Delete all waypoints clicked - Cleared mission %d", current_mission);
      }
    }

    std::map<int, Ogre::SceneNode* >::iterator sn_it;
    for (sn_it = waypointNodeMap_->begin(); sn_it != waypointNodeMap_->end(); sn_it++) {
      if (scene_manager_ && sn_it->second) {
        scene_manager_->destroySceneNode(sn_it->second);
      }
    }
    
    waypointNodeMap_->clear();
    *uniqueWaypointIndex_ = 0;
    if (server_) {
      server_->clear();
      server_->applyChanges();
    }
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Clear waypoint success");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in deleteAllWaypoints function: %s", e.what());
  }
}

void WaypointNavPanel::deleteSelectedWaypoint() {
  try {
    std::cout << "delete wp entry" << std::endl;
    int from_wp = ui_->delete_wp_from->value() - 1;
    int to_wp = ui_->delete_wp_to->value();

    std::cout << "delete wp entry 2" << std::endl;
    if (from_wp < 0 || to_wp > paths[current_mission].poses.size() || from_wp > to_wp) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Invalid range to delete waypoints");
      return;
    }
    
    // Delete the waypoints from the stored path
    paths[current_mission].poses.erase(paths[current_mission].poses.begin() + from_wp, 
                                       paths[current_mission].poses.begin() + to_wp);
    
    // Clear visual markers without clearing path data
    deleteAllWaypoints(false);
    
    // Reset marker_id before redrawing
    wp_nav_tool_->marker_id_ = 0;
    
    // Redraw only the current mission to reflect changes
    drawPath(current_mission);
    
    mission_str = std::to_string((current_mission+1)/2);
    mission_str += (current_mission%2 ? " pre" : " in");
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), 
                "Successfully deleted waypoints %d to %d in mission %s", 
                from_wp + 1, to_wp, mission_str.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in deleteSelectedWaypoint function: %s", e.what());
  }
}

void WaypointNavPanel::placeWaypoint() {
  try {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform("map", "asv/base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "TF Exception: %s", ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped temp;
    temp.pose.position.x = transformStamped.transform.translation.x;
    temp.pose.position.y = transformStamped.transform.translation.y;
    temp.pose.position.z = default_height_;

    // Ignore if position x and y are both 0
    if (temp.pose.position.x == 0.0 && temp.pose.position.y == 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Waypoint with position (0, 0) ignored");
        return;
    }

    temp.pose.orientation.w = transformStamped.transform.rotation.w;
    temp.pose.orientation.x = transformStamped.transform.rotation.x;
    temp.pose.orientation.y = transformStamped.transform.rotation.y;
    temp.pose.orientation.z = transformStamped.transform.rotation.z;

    paths[current_mission].poses.push_back(temp);
    mission_str = std::to_string((current_mission+1)/2);
    mission_str += (current_mission%2 ? " pre" : " in");
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Successfully inserted waypoint in mission %s", mission_str.c_str());

    // Clear visual markers without clearing path data
    deleteAllWaypoints(false);
    
    // Reset marker_id before redrawing
    wp_nav_tool_->marker_id_ = 0;
    
    // Redraw the current mission
    drawPath(current_mission, false);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in placeWaypoint function: %s", e.what());
  }
}

void WaypointNavPanel::insertWaypoint(const Ogre::Vector3& pos, const Ogre::Quaternion& quat) {
  try {
    mission_str = std::to_string((current_mission + 1) / 2);
    mission_str += (current_mission % 2 ? " pre" : " in");

    int index_to_insert;
    if (current_mission == 0) {
      index_to_insert = 0;
      // Don't clear path data, just refresh the visual markers
      deleteAllWaypoints(false);
      // Reset marker_id before redrawing all missions
      wp_nav_tool_->marker_id_ = 0;
      for (int i = 1; i < 21; i++) {
        drawPath(i, true);
      }
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Cannot insert waypoint in 'Select Mission' mode. Please select a specific mission.");
    } else {
      index_to_insert = paths[current_mission].poses.size();
      geometry_msgs::msg::PoseStamped tmp_wp;
    
    tmp_wp.pose.position.x = pos.x;
    tmp_wp.pose.position.y = pos.y;
    tmp_wp.pose.position.z = pos.z;

    tmp_wp.pose.orientation.w = quat.w;
    tmp_wp.pose.orientation.x = quat.x;
    tmp_wp.pose.orientation.y = quat.y;
    tmp_wp.pose.orientation.z = quat.z;

    // Check the distance with the last waypoint
    // if (index_to_insert > 0 && ui_->insert_mode_checkBox->isChecked()) {
    //   const auto& last_wp = paths[current_mission].poses.back().pose.position;
    //   double dx = pos.x - last_wp.x;
    //   double dy = pos.y - last_wp.y;
    //   double dz = pos.z - last_wp.z;
    //   double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    //   if (distance > 5.0) {
    //     int num_intermediate_points = static_cast<int>(std::floor(distance / 5.0));
    //     for (int i = 1; i <= num_intermediate_points; ++i) {
    //       double ratio = static_cast<double>(i) / (num_intermediate_points + 1);

    //       geometry_msgs::msg::PoseStamped intermediate_wp;
    //       intermediate_wp.pose.position.x = last_wp.x + dx * ratio;
    //       intermediate_wp.pose.position.y = last_wp.y + dy * ratio;
    //       intermediate_wp.pose.position.z = last_wp.z + dz * ratio;

    //       intermediate_wp.pose.orientation = tmp_wp.pose.orientation; // Same orientation

    //       paths[current_mission].poses.push_back(intermediate_wp);
    //     }
    //   }
    // }

    // Insert the new waypoint into the stored path
    paths[current_mission].poses.push_back(tmp_wp);
    
    // Clear visual markers without clearing path data
    deleteAllWaypoints(false);
    
    // Reset marker_id before redrawing
    wp_nav_tool_->marker_id_ = 0;
    
    // Redraw the current mission to show the new waypoint
    drawPath(current_mission, false);
    
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), 
                "Successfully created waypoint %d in mission %s", 
                index_to_insert + 1, mission_str.c_str());
    }

    // Update UI components
    ui_->insert_wp_spinBox->setValue(paths[current_mission].poses.size());
    ui_->insert_wp_spinBox->setMinimum(1);
    ui_->insert_wp_spinBox->setMaximum(paths[current_mission].poses.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in insertWaypoint function: %s", e.what());
  }
}

void WaypointNavPanel::publishAllWaypoints() {
  // Check if ROS is still running
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "ROS is not running, cannot publish waypoints");
    return;
  }

  // Check if client_node is available
  if (!client_node_) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Client node is not available, cannot publish waypoints");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Starting to publish all waypoints...");
  
  int successful_publishes = 0;
  int failed_publishes = 0;

  // using service
  for(int i = 0; i < 20; i++) {
    try {
      mission_str = std::to_string((i/2)+1);
      mission_str += (((i/2)+1)%2? " pre" : " in");
      
      // Check if we have waypoints for this mission (use correct index)
      if(paths[i+1].poses.size() > 0) {
        // Check if service client exists and is valid
        if (path_client_[i+1] && path_client_[i+1]->service_is_ready()) {
          // Wait longer for service availability during initialization
          if (path_client_[i+1]->wait_for_service(std::chrono::seconds(2))) {
            auto request = std::make_shared<nala2_interfaces::srv::SetPath::Request>();
            request->path = paths[i+1];  // Use correct index

            // Send async request without immediately calling get()
            auto future_result = path_client_[i+1]->async_send_request(request);
            
            // Wait for response with timeout
            auto status = future_result.wait_for(std::chrono::seconds(3));
            if (status == std::future_status::ready) {
              try {
                auto response = future_result.get();
                if (response && response->success) {
                  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Mission %s published successfully", mission_str.c_str());
                  successful_publishes++;
                } else {
                  RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Mission %s service call failed: %s", 
                             mission_str.c_str(), response ? response->message.c_str() : "Unknown error");
                  failed_publishes++;
                }
              } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Exception getting response for mission %s: %s", 
                           mission_str.c_str(), e.what());
                failed_publishes++;
              }
            } else {
              RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Timeout waiting for response from mission %s service", mission_str.c_str());
              failed_publishes++;
            }
          } else {
            RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Service for mission %s not available after waiting", mission_str.c_str());
            failed_publishes++;
          }
        } else {
          RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Service client for mission %s is not ready", mission_str.c_str());
          failed_publishes++;
        }
      } else {
        RCLCPP_DEBUG(rclcpp::get_logger("waypoint_panel"), "Mission %s has no waypoints, skipping", mission_str.c_str());
      }
      
      // Add small delay between service calls to prevent overwhelming the system
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error publishing mission %s: %s", mission_str.c_str(), e.what());
      failed_publishes++;
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Finished publishing waypoints - Success: %d, Failed: %d", 
             successful_publishes, failed_publishes);
}

void WaypointNavPanel::publishWaypoint() {
  try {
    // using topic
    // wp_pub_[current_mission]->publish(paths[current_mission]);

    // using service
    if (!path_client_[current_mission] || !path_client_[current_mission]->wait_for_service(0.25s)) {
      mission_str = std::to_string((current_mission/2)+1);
      mission_str = (((current_mission/2)+1)%2? " pre" : " in");
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "service for mission %s not available", mission_str.c_str());
      return;
    }
    auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::SetPath::Response>>(); 

    auto request = std::make_shared<nala2_interfaces::srv::SetPath::Request>();
    request->path = paths[current_mission];

    future_result = path_client_[current_mission]->async_send_request(request);
    auto response = future_result.get();
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Response: %s, Success: %s",
                response->message.c_str(), response->success ? "true" : "false");

    mission_str = std::to_string((current_mission/2)+1);
    ((current_mission/2)+1)%2? mission_str += " pre" : mission_str += " in";
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Mission %s have been called", mission_str.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error publishing waypoint: %s", e.what());
  }
}

void WaypointNavPanel::setSelectedMarkerName(std::string name)
{
  try {
    std::stringstream wp_label_name;
    mission_str = std::to_string((current_mission+1)/2);
    mission_str += (current_mission%2? " pre" : " in");
    
    wp_label_name << name << " , mission "<< current_mission;

    selected_marker_name_ = wp_label_name.str();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in setSelectedMarkerName function: %s", e.what());
  }
}

void WaypointNavPanel::poseChanged(double val) {
  try {
    auto sn_entry = waypointNodeMap_->end();
    try {
      const int selected_marker_idx=0;
      std::stoi(selected_marker_name_.substr(strlen(wp_name_prefix), 2));
      sn_entry = waypointNodeMap_->find(selected_marker_idx);
    } catch (const std::logic_error &e) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), e.what());
      return;
    }

    if (sn_entry == waypointNodeMap_->end())
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "%s not found in map", selected_marker_name_.c_str());
    else {
      Ogre::Vector3 position;
      Ogre::Quaternion quat;
      getPose(position, quat);

      sn_entry->second->setPosition(position);
      sn_entry->second->setOrientation(quat);

      // Also update the visual child node to ensure it stays synchronized
      std::map<int, Ogre::SceneNode*>::iterator visual_entry = waypointNodeMap_->find(sn_entry->first + 1000);
      if (visual_entry != waypointNodeMap_->end()) {
        // Ensure the visual child maintains its offset rotation
        visual_entry->second->setOrientation(Ogre::Quaternion(Ogre::Degree(-93.8), Ogre::Vector3::UNIT_Z));
      }

      std::stringstream wp_name;
      wp_name << wp_name_prefix << sn_entry->first;
      std::string wp_name_str(wp_name.str());

      if(server_ && server_->get(wp_name_str, int_marker))
      {
        int_marker.pose.position.x = position.x;
        int_marker.pose.position.y = position.y;
        int_marker.pose.position.z = position.z;

        int_marker.pose.orientation.x = quat.x;
        int_marker.pose.orientation.y = quat.y;
        int_marker.pose.orientation.z = quat.z;
        int_marker.pose.orientation.w = quat.w;

        server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
      }
      if (server_) {
        server_->applyChanges();
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in poseChanged function: %s", e.what());
  }
}

void WaypointNavPanel::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  try {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    position.x = ui_->x_doubleSpinBox->value();
    position.y = ui_->y_doubleSpinBox->value();
    position.z = ui_->z_doubleSpinBox->value();
    double yaw = ui_->yaw_doubleSpinBox->value();

    tf2::Quaternion qt;
    qt.setRPY(0, 0, yaw);
    quat.x = qt.x();
    quat.y = qt.y();
    quat.z = qt.z();
    quat.w = qt.w();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in getPose function: %s", e.what());
  }
}

void WaypointNavPanel::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quat, const int wp_index)
{
  try {
    ui_->x_doubleSpinBox->blockSignals(true);
    ui_->y_doubleSpinBox->blockSignals(true);
    ui_->z_doubleSpinBox->blockSignals(true);
    ui_->yaw_doubleSpinBox->blockSignals(true);

    ui_->x_doubleSpinBox->setValue(position.x);
    ui_->y_doubleSpinBox->setValue(position.y);
    ui_->z_doubleSpinBox->setValue(position.z);

    double roll, pitch, yaw;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    ui_->yaw_doubleSpinBox->setValue(yaw);

    //enable the signals
    ui_->x_doubleSpinBox->blockSignals(false);
    ui_->y_doubleSpinBox->blockSignals(false);
    ui_->z_doubleSpinBox->blockSignals(false);
    ui_->yaw_doubleSpinBox->blockSignals(false);

    mission_str = std::to_string((current_mission+1)/2);
    mission_str += (current_mission%2? " pre" : " in");

    // std::cout << "wp index: " << wp_index << " | total wp: " << paths[current_mission].poses.size() << std::endl;
    if (wp_index >= 0 && (wp_index-1) < paths[current_mission].poses.size()) {
      paths[current_mission].poses[wp_index-1].pose.position.x = position.x;
      paths[current_mission].poses[wp_index-1].pose.position.y = position.y;
      paths[current_mission].poses[wp_index-1].pose.position.z = position.z;
      
      paths[current_mission].poses[wp_index-1].pose.orientation.x = quat.x;
      paths[current_mission].poses[wp_index-1].pose.orientation.y = quat.y;
      paths[current_mission].poses[wp_index-1].pose.orientation.z = quat.z;
      paths[current_mission].poses[wp_index-1].pose.orientation.w = quat.w;

      paths[current_mission].poses[wp_index-1].header.frame_id = "map";
      paths[current_mission].poses[wp_index-1].header.stamp = client_node_->now();
      // RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "updated waypoint %d on mission %s", (wp_index-1), mission_str.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in setPose function: %s", e.what());
  }
}

void WaypointNavPanel::setWpLabel() {
  try {
    std::ostringstream stringStream;
    stringStream << selected_marker_name_;
    std::string label = stringStream.str();

    ui_->sel_wp_label->setText(QString::fromStdString(label));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in setWpLabel function: %s", e.what());
  }
}

void WaypointNavPanel::saveWaypoint() {
  try {
    ui_->save_wp_button->setEnabled(false);

    // If there is nothing to save, inform the user and return early
    size_t total_poses = 0;
    for (size_t i = 1; i < paths.size(); ++i) {
      total_poses += paths[i].poses.size();
    }
    if (total_poses == 0) {
      QMessageBox::information(this, tr("Save Waypoints"), tr("No waypoints to save."));
      ui_->save_wp_button->setEnabled(true);
      return;
    }

    // Automated save: predefined directory in user's home
    const QString save_dir = QDir::homePath() + "/nala2/waypoints";
    QDir().mkpath(save_dir);

    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    const QString filename = QString("waypoints_%1.yaml").arg(timestamp);
    const QString path = QDir(save_dir).filePath(filename);

    const std::string path_str = path.toStdString();
    RVIZ_COMMON_LOG_INFO_STREAM("Saving waypoints to " << path_str);
    writeToYaml(path_str);

    ui_->save_wp_button->setEnabled(true);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in saveWaypoint function: %s", e.what());
    ui_->save_wp_button->setEnabled(true);
  }
}

void WaypointNavPanel::writeToYaml(const std::string &filename) {
  try {
    YAML::Emitter out;  
    // Add a non-invasive time log as a YAML comment so loaders remain compatible
    const QString iso_time = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    out << YAML::Comment(std::string("saved_at_utc: ") + iso_time.toStdString());
    out << YAML::BeginMap;
    for (int i=1; i<paths.size(); i++) {
      nav_msgs::msg::Path &tempPath = paths[i];

      // Use a unique key per mission to avoid overwriting in YAML maps
      out << YAML::Key << (std::string("Mission_") + std::to_string(i));
      out << YAML::Value;
      out << YAML::BeginMap;

      int pose_idx = 0;
      for (const auto& pos : tempPath.poses) {
        out << YAML::Key << ("Pose" + std::to_string(pose_idx++));
        out << YAML::Value;
        out << YAML::Flow;
        out << YAML::BeginSeq;
        out << pos.pose.position.x;
        out << pos.pose.position.y;
        out << pos.pose.position.z;
        out << pos.pose.orientation.x;
        out << pos.pose.orientation.y;
        out << pos.pose.orientation.z;
        out << pos.pose.orientation.w;
        out << YAML::EndSeq;
      }
      out << YAML::EndMap;  
    }
    out << YAML::EndMap;

    std::ofstream fout(filename);
    if (fout.is_open()) {
      fout << out.c_str();
      fout.close();
      RCLCPP_INFO(rclcpp::get_logger("waypoint_nav_frame"), "Successfully wrote waypoints to YAML file: %s", filename.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_frame"), "Could not open file for writing: %s", filename.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_frame"), "Error in writeToYaml function: %s", e.what());
  }
}

void WaypointNavPanel::writeToJson(const std::string &filename) {
  try {
    std::ofstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_frame"), "Could not open JSON file for writing: %s", filename.c_str());
      return;
    }

    json allPathsMission;
    for (int i=1; i<paths.size(); i++) { 
        nav_msgs::msg::Path *tempPath = &paths[i];
        json pathDetail;
        for (int a = 0; a < tempPath->poses.size(); a++) { 
            json poseDetail;
            poseDetail["x"] = tempPath->poses[a].pose.position.x;
            poseDetail["y"] = tempPath->poses[a].pose.position.y;
            poseDetail["z"] = tempPath->poses[a].pose.position.z;
            poseDetail["qx"] = tempPath->poses[a].pose.orientation.x;
            poseDetail["qy"] = tempPath->poses[a].pose.orientation.y;
            poseDetail["qz"] = tempPath->poses[a].pose.orientation.z;
            poseDetail["qw"] = tempPath->poses[a].pose.orientation.w;
            pathDetail.push_back(poseDetail);
        }
        allPathsMission.push_back(pathDetail);
    }
    file << allPathsMission;
    file.close();
    RCLCPP_INFO(rclcpp::get_logger("waypoint_nav_frame"), "Successfully wrote waypoints to JSON file: %s", filename.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_frame"), "Error in writeToJson function: %s", e.what());
  }
}

void WaypointNavPanel::loadWaypoint() {
  try {
    ui_->load_wp_button->setEnabled(false);

    // Open from the same home directory used for automated saves
    const QString load_dir = QDir::homePath() + "/nala2/waypoints";
    QDir().mkpath(load_dir);
    QString path = QFileDialog::getOpenFileName(
      this, tr("Load Waypoints"), load_dir,
      tr("Waypoint Files (*.yaml *.json)"));

    if (path.isEmpty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("waypoint_panel"), 
        "Failed to load Waypoint: Path Input Error");
      ui_->load_wp_button->setEnabled(true);
      return;
    }

    const std::string path_str = path.toStdString();
    RVIZ_COMMON_LOG_INFO_STREAM("loading waypoints from " << path_str);
    if (path.endsWith(".json")) readFromJson(path_str);
    else readFromYaml(path_str);

    ui_->load_wp_button->setEnabled(true);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in loadWaypoint function: %s", e.what());
    ui_->load_wp_button->setEnabled(true);
  }
}

void WaypointNavPanel::readFromYaml(const std::string &filename) {
  try {
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Start load file yaml");

    // Clear any existing visual markers and stored paths before loading new file
    // This ensures we don't mix previously loaded/created waypoints with the newly loaded ones
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Clearing existing waypoints before loading");
    // Clear visual markers and interactive markers (does not clear stored paths when called with false)
    deleteAllWaypoints(false);
    // Clear stored paths for all missions
    for (size_t idx = 1; idx < paths.size(); ++idx) {
      paths[idx].poses.clear();
    }
    // Reset waypoint indices used by tools and markers
    if (uniqueWaypointIndex_) {
      *uniqueWaypointIndex_ = 0;
    }
    if (wp_nav_tool_) {
      wp_nav_tool_->marker_id_ = 0;
    }

    YAML::Node file = YAML::LoadFile(filename);
    paths[0].poses.clear();
    int i=1;

    for (const auto& temp_mission : file) {
      std::string mission_name = temp_mission.first.as<std::string>();
      YAML::Node mission = temp_mission.second;

      if (!mission.IsNull()) {
        int j=0;      

        for (const auto& temp_pose : mission) {
          std::string pose_name = temp_pose.first.as<std::string>();
          auto pose = temp_pose.second.as<std::vector<double>>();

          if (pose.size() == 7) {
            geometry_msgs::msg::PoseStamped pos;
            pos.pose.position.x = pose[0];
            pos.pose.position.y = pose[1];
            pos.pose.position.z = pose[2];
            pos.pose.orientation.x = pose[3];
            pos.pose.orientation.y = pose[4];
            pos.pose.orientation.z = pose[5];
            pos.pose.orientation.w = pose[6];
            paths[i].poses.push_back(pos);
            j++;
          } 
        }
      }
      i++;
    }
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Load file yaml complete");  
    
    // 0.1 second delay per mission ensures proper rendering
    for (int mission_idx = 1; mission_idx <= 20; mission_idx++) {
      ui_->missionComboBox->setCurrentIndex(mission_idx);
      setMission(mission_idx);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Finally switch to "Select Mission" to show all missions
    ui_->missionComboBox->setCurrentIndex(0);
    setMission(0);
    
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "All missions initialized and displayed");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in readFromYaml function: %s", e.what());
  }
}

void WaypointNavPanel::readFromJson(const std::string &filename) {
  try {
    QString buffer;

    // Clear any existing visual markers and stored paths before loading new file
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Clearing existing waypoints before loading JSON");
    deleteAllWaypoints(false);
    for (size_t idx = 1; idx < paths.size(); ++idx) {
      paths[idx].poses.clear();
    }
    if (uniqueWaypointIndex_) {
      *uniqueWaypointIndex_ = 0;
    }
    if (wp_nav_tool_) {
      wp_nav_tool_->marker_id_ = 0;
    }

    std::ifstream openFile(filename);
    if (!openFile.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Could not open JSON file for reading: %s", filename.c_str());
      return;
    }

    json allPathMissionLoad;
    openFile >> allPathMissionLoad;
    paths[0].poses.clear();
    for(int i = 0; i < allPathMissionLoad.size(); i++) {
        if(!allPathMissionLoad[i].is_null()) {
            json pathDetail = allPathMissionLoad[i];
            paths[i+1].poses.clear();
            for(int a = 0; a <pathDetail.size(); a++) {
                json poseDetail = pathDetail[a];
                geometry_msgs::msg::PoseStamped pos;
                pos.pose.position.x = poseDetail["x"];
                pos.pose.position.y = poseDetail["y"];
                pos.pose.position.z = poseDetail["z"];
                pos.pose.orientation.x = poseDetail["qx"];
                pos.pose.orientation.y = poseDetail["qy"];
                pos.pose.orientation.z = poseDetail["qz"];
                pos.pose.orientation.w = poseDetail["qw"];
                paths[i+1].poses.push_back(pos);                
            }
        }
    }
    
    // Cycle through each mission sequentially to initialize all visual markers
    // 0.1 second delay per mission ensures proper rendering
    for (int mission_idx = 1; mission_idx <= 20; mission_idx++) {
      ui_->missionComboBox->setCurrentIndex(mission_idx);
      current_mission = mission_idx;
      setMission(mission_idx);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Finally switch to "Select Mission" to show all missions
    ui_->missionComboBox->setCurrentIndex(0);
    current_mission = 0;
    setMission(0);
    
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Successfully loaded waypoints from JSON file: %s", filename.c_str());
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "All missions initialized and displayed");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in readFromJson function: %s", e.what());
  }
}

void WaypointNavPanel::autonomy() {
  try {
    const QString button_style_template = "QPushButton {"
                                          "border-radius: 20px;"
                                          "background-color: %1;"
                                          "color: black;"
                                          "border: 1px solid black;"
                                          "padding: 10px 20px;"
                                          "}";

    const QString start_color = "#33b249";
    const QString stop_color = "#ED0800";

    if (!autonomy_client_ || !autonomy_client_->wait_for_service(0.25s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "service for starting mission not available");
      return;
    }
    auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::StringService::Response>>();  

    QString command = ui_->command_button->text();
    if (command.contains("start", Qt::CaseInsensitive)) {
      ui_->command_button->setText("Stop Autonomy");
      ui_->command_button->setStyleSheet(button_style_template.arg(stop_color));

      auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
      request->data = "start";

      auto future_result = autonomy_client_->async_send_request(request);
      RCLCPP_INFO(client_node_->get_logger(), "Service to start the mission is called");
    } 
    else {
      ui_->command_button->setText("Start Autonomy");
      ui_->command_button->setStyleSheet(button_style_template.arg(start_color));

      auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
      request->data = "stop";

      auto future_result = autonomy_client_->async_send_request(request);
      RCLCPP_INFO(client_node_->get_logger(), "Service to stop the mission is called");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in autonomy function: %s", e.what());
  }
}

void WaypointNavPanel::recordRoute() {
  try {
    const QString button_style_template = "QPushButton {"
    "border-radius: 20px;"
    "background-color: %1;"
    "color: black;"
    "border: 1px solid black;"
    "padding: 10px 10px;"
    "}";

    const QString start_color = "rgb(152, 106, 68)";
    const QString stop_color = "#ED0800";

    if (!route_client_ || !route_client_->wait_for_service(0.25s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
        RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "service for recording route not available");
        return;
    }
    auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::StringService::Response>>();  

    QString command = ui_->start_route_button->text();
    if (command.contains("start", Qt::CaseInsensitive)) {
      ui_->start_route_button->setText("Stop Route");
      ui_->start_route_button->setStyleSheet(button_style_template.arg(stop_color));

      auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
      request->data = "start";

      auto future_result = route_client_->async_send_request(request);
      RCLCPP_INFO(client_node_->get_logger(), "Service to start the recording of the route is called");
    } 
    else {
      ui_->start_route_button->setText("Start Route");
      ui_->start_route_button->setStyleSheet(button_style_template.arg(start_color));

      auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
      request->data = "stop";

      auto future_result = route_client_->async_send_request(request);
      RCLCPP_INFO(client_node_->get_logger(), "Service to stop the recording of the route is called");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in recordRoute function: %s", e.what());
  }
}

void WaypointNavPanel::clearRoute() {
  try {
    if (!route_client_ || !route_client_->wait_for_service(0.25s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "service for clearing route not available");
      return;
    }

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "clear";

    auto future_result = route_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to clear the recorded route is called");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in clearRoute function: %s", e.what());
  }
}


void WaypointNavPanel::saveRoute() {
  try {
    if (!route_client_ || !route_client_->wait_for_service(0.25s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "service for save route not available");
      return;
    }

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "save";

    auto future_result = route_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to save the recorded route is called");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in saveRoute function: %s", e.what());
  }
}


void WaypointNavPanel::loadRoute() {
  try {
    if (!route_client_ || !route_client_->wait_for_service(0.25s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "service for load route not available");
      return;
    }

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "load";

    auto future_result = route_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to load the recorded route is called");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in loadRoute function: %s", e.what());
  }
}

double WaypointNavPanel::getDefaultHeight() {
  try {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    return default_height_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in getDefaultHeight function: %s", e.what());
    return 0.0;
  }
}

QString WaypointNavPanel::getFrameId() {
  try {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    return frame_id_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in getFrameId function: %s", e.what());
    return QString();
  }
}

QString WaypointNavPanel::getOutputTopic() {
  try {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    return output_topic_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in getOutputTopic function: %s", e.what());
    return QString();
  }
}

rclcpp::Node::SharedPtr WaypointNavPanel::createNewNode(const std::string & node_name) {
  try {
    std::string node = "__node:=" + node_name;
    auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node, "--"});
    return std::make_shared<rclcpp::Node>("_", options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in createNewNode function: %s", e.what());
    return nullptr;
  }
}

void WaypointNavPanel::timerEvent(QTimerEvent * event) {
  try {
    if (event && event->timerId() == timer_id_ && client_node_) {
      rclcpp::spin_some(client_node_);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Error in timerEvent: %s", e.what());
  }
}
} // namespace

PLUGINLIB_EXPORT_CLASS(waypoint_nav_plugin::WaypointNavPanel, rviz_common::Panel)