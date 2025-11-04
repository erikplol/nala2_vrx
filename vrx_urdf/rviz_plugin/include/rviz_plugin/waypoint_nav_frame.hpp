#ifndef KR_RVIZ_PLUGIN_WAYPOINT_FRAME_
#define KR_RVIZ_PLUGIN_WAYPOINT_FRAME_

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif
#include <vector>
#include <thread>
#include <future>

#include <rviz_common/panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

#include <Ogre.h>
#include <QKeyEvent>
#include <interactive_markers/interactive_marker_server.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rviz_plugin/json.hpp>

#include <nala2_interfaces/srv/string_service.hpp>
#include <nala2_interfaces/srv/set_path.hpp>

#include <fstream>
#include <OgreSceneNode.h>
#include <chrono>

#include "waypoint_nav_tool.hpp"

#include <QTimer>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <QFileDialog>
#include <QKeyEvent>

#include <boost/foreach.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "ui_WaypointNavigation.h"

namespace Ogre 
{
class SceneNode;
}

namespace Ui
{
class WaypointNavigationWidget;
}

namespace waypoint_nav_plugin
{
class WaypointNavTool;
class WaypointNavPanel;
}

namespace waypoint_nav_plugin
{

class WaypointNavTool;
constexpr char wp_name_prefix[] = "waypoint_";

class WaypointNavPanel : public rviz_common::Panel, public std::enable_shared_from_this<WaypointNavPanel>
{
  friend class WaypointNavTool;
  Q_OBJECT

public:
  WaypointNavPanel(QWidget *parent = nullptr
                  , rviz_common::DisplayContext* context = NULL
                  , std::map<int, Ogre::SceneNode* >* waypointNodeMap = nullptr
                  , std::shared_ptr<interactive_markers::InteractiveMarkerServer> server = nullptr
                  , std::shared_ptr<int> uniqueWaypointIndex = nullptr
                  , WaypointNavTool* wp_tool = nullptr
                  );
  ~WaypointNavPanel();

protected:
  std::shared_ptr<Ui::WaypointNavigationWidget> ui_;
  rviz_common::DisplayContext *context_;
  rclcpp::Node::SharedPtr createNewNode(const std::string & node_name);
  
  void enable();
  void disable();

  void initPublisher();
  void initService();

  void drawPath(int current_mission, bool is_base_mission=false);
  void poseCallback(const geometry_msgs::msg::PoseStamped& msg);

  void setPath();
  void setWpCount(int size);
  void setWpLabel();
  void setSelectedMarkerName(std::string name);
  void setConfig(QString topic, QString frame, float height);
  void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quat, const int wp_index);
  
  void writeToYaml(const std::string &filename);
  void writeToJson(const std::string &filename);
  void readFromYaml(const std::string &filename);
  void readFromJson(const std::string &filename);
  
  QString getFrameId();
  QString getOutputTopic();
  double getDefaultHeight();
  void getPose(Ogre::Vector3& position, Ogre::Quaternion& quat);
  
  private Q_SLOTS:
  void setMission(int index);
  void poseChanged(double val);
  
  void autonomy();
  void recordRoute();
  void clearRoute();
  void loadRoute();
  void saveRoute(); 
  void spinOnce();
  void timerEvent(QTimerEvent * event) override;

  void calibratePose(bool button_clicked);

  void placeWaypoint();
  void insertWaypoint(const Ogre::Vector3& pos, const Ogre::Quaternion& quat);
  
  void publishAllWaypoints();
  void publishWaypoint();
  
  void saveWaypoint();
  void loadWaypoint();

  void deleteSelectedWaypoint();
  void deleteAllWaypoints(bool button_clicked = false);
  void clearAllWaypoints(bool button_clicked = false);

private:
  Ogre::SceneManager* scene_manager_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wp_pub_[27];
  rclcpp::Client<nala2_interfaces::srv::SetPath>::SharedPtr path_client_[27];
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Client<nala2_interfaces::srv::StringService>::SharedPtr autonomy_client_;
  rclcpp::Client<nala2_interfaces::srv::StringService>::SharedPtr route_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_client_;


  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path nala_path_;
  std::shared_ptr<WaypointNavTool> wp_nav_tool_;
  visualization_msgs::msg::InteractiveMarker int_marker;

  
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* waypointNodeMap_;
  std::shared_ptr<std::thread> spin_thread_;
  int timer_id_; 

  // mission
  int is_clear = 1;
  int current_mission;
  int selected_waypoint;
  std::string mission_str;
  std::string current_topic;
  std::shared_ptr<int> uniqueWaypointIndex_;
  std::vector<nav_msgs::msg::Path> paths = std::vector<nav_msgs::msg::Path>(27);

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  //default height the waypoint must be placed at
  double default_height_;

  // The current name of the output topic.
  QString frame_id_;
  QString output_topic_;

  //mutex for changes of variables
  boost::mutex frame_updates_mutex_;
  std::string selected_marker_name_;
};

}

#endif