
#pragma once

#include <memory>
#include <random>
#include <iostream>
#include <stdlib.h>
#include <time.h>

// ros
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Empty.h>
// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_object.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/objects/aruco_gate.hpp"
#include "flightlib/objects/object_base.hpp"
#include "flightlib/objects/dynamic_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"


using namespace flightlib;

namespace flightmare_px4 {

class FlightmarePX4 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightmarePX4(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightmarePX4();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void resetCallback(std_msgs::Empty sig);
  void netCallback(std_msgs::Empty sig);

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  sensor_msgs::CameraInfo camera_info_msg_; 
  
  // publisher
  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher segmentation_pub_;
  image_transport::Publisher opticalflow_pub_;

  ros::Publisher camera_info_pub_;
  ros::Publisher gate_pose_pub;
  
  int contador = 0;
  
  // subscriber
  ros::Subscriber sub_state_est_;
  ros::Subscriber env_reset_;
  ros::Subscriber net_update_;
  
  // service client
  ros::ServiceClient pause_srv_, unpause_srv_;
  
  //tf broadcaster
  tf::TransformBroadcaster br_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  std::shared_ptr<Quadrotor> quad_ptr_1;
  std::shared_ptr<RGBCamera> rgb_camera_1;
  std::shared_ptr<StaticGate> gate;
  std::shared_ptr<StaticGate> gate2;
  std::shared_ptr<StaticGate> gate3;
  std::shared_ptr<StaticGate> gate4;
  std::shared_ptr<StaticGate> gate5;
  std::shared_ptr<StaticGate> gate6;
  std::shared_ptr<StaticGate> gate7;
  std::shared_ptr<StaticGate> gate8;
  std::shared_ptr<StaticGate> gate9;
  QuadState quad_state_;
  QuadState quad_state_1;
  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  bool unity_render_1{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};  

  // auxiliary variables
  double main_loop_freq_;
  tf::Transform camera_transform_;
  std::string frame_id_;
  int render_id;
  std::string drone_name;
  std::string p_port;
  std::string s_port;
  bool net_updating;
  
  bool enable_depth_, enable_segmentation_, enable_opt_flow_;
};
}  // namespace flightmare_px4
