#include "flightmare_px4/FlightmarePX4.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>

namespace flightmare_px4 {

FlightmarePX4::FlightmarePX4(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::ARUCO),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(1),
    main_loop_freq_(60.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }
  // quad initialization
  drone_name = pnh_.getNamespace().c_str();
  drone_name = drone_name.substr(1, 6);
  std::cout << drone_name << std::endl;

  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
/*  double cam_x, cam_y, cam_z, cam_qx, cam_qy, cam_qz, cam_qw; 
  pnh_.param("cam_x", cam_x, 0.3);
  pnh_.param("cam_y", cam_y, 0.);
  pnh_.param("cam_z", cam_z, 0.3);
  pnh_.param("cam_qx", cam_qx, 0.5);
  pnh_.param("cam_qy", cam_qy, -0.5);
  pnh_.param("cam_qz", cam_qz, 0.5);
  pnh_.param("cam_qw", cam_qw, -0.5);
  tf::Vector3 cam_origin(cam_x, cam_y, cam_z);
  tf::Quaternion cam_rot(cam_qx, cam_qy, cam_qz, cam_qw); //(0.4196665, -0.4196665, 0.5691046, -0.5691046); //(0.5, -0.5, 0.5, -0.5);  
  
  tf::Transform flu2rfu(tf::Quaternion(0,0,0.7071068,0.7071068));
  tf::Transform rotTransfo(tf::Quaternion(0.5, -0.5, 0.5, 0.5));
  
  tf::Quaternion cam_rot_rfu = flu2rfurotTransfo*cam_rot;
  tf::Vector3 cam_origin_rfu = flu2rfu*cam_origin;
  
  camera_transform_ = tf::Transform(cam_rot, cam_origin); */

  /*rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(0.7071068, 0.0, 0.0, -0.7071068).toRotationMatrix();
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(640);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_); */
  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  //image_transport::ImageTransport it_(pnh_);
  //it_.reset(new image_transport::ImageTransport(nh_));
  //rgb_pub_ = it_.advertise("camera/rgb", 1);
  gate_pose_pub = nh_.advertise<geometry_msgs::PoseArray>("/" + drone_name +"/flightmare/gate_position", 1, true);
  
  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("/"+ drone_name+"/self_localization/pose", 1,
                                 &FlightmarePX4::poseCallback, this);

  env_reset_ = nh_.subscribe("/"+ drone_name +"/environment/flightmare/reset", 1,
                                 &FlightmarePX4::resetCallback, this);

  net_update_ = nh_.subscribe("/environment/flightmare/net_update", 1, &FlightmarePX4::netCallback, this);
  
  pause_srv_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  unpause_srv_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  
  timer_main_loop_ = nh_.createTimer(ros::Rate(30.0),
                                     &FlightmarePX4::mainLoopCallback, this);

  net_updating = false;

    std::string object_id = "unity_gate1";
    std::string prefab_id = "aruco_gate2";
    gate = std::make_shared<StaticGate>(object_id, prefab_id);
    gate->setPosition(Eigen::Vector3f(6, 0, 0));
    gate->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 1.0));
    /*std::string object_id2 = "unity_gate2";
    std::string prefab_id2 = "aruco_gate2";
    gate2 = std::make_shared<StaticGate>(object_id2, prefab_id2);
    gate2->setPosition(Eigen::Vector3f(10, 0, 0));
    gate2->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 1.0));
    std::string object_id3 = "unity_gate3";
    std::string prefab_id3 = "aruco_gate2";
    gate3 = std::make_shared<StaticGate>(object_id3, prefab_id3);
    gate3->setPosition(Eigen::Vector3f(14, 0, 0));
    gate3->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 1.0));
    std::string object_id4 = "unity_gate4";
    std::string prefab_id4 = "aruco_gate2";
    gate4 = std::make_shared<StaticGate>(object_id4, prefab_id4);
    gate4->setPosition(Eigen::Vector3f(16, -2, 0));
    gate4->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 1.0));
    std::string object_id5 = "unity_gate5";
    std::string prefab_id5 = "aruco_gate2";
    gate5 = std::make_shared<StaticGate>(object_id5, prefab_id5);
    gate5->setPosition(Eigen::Vector3f(17, -4, 0));
    gate5->setQuaternion(
    Quaternion(0.7071068, 0.0, 0.0, 0.7071068));
    std::string object_id6 = "unity_gate6";
    std::string prefab_id6 = "aruco_gate2";
    gate6 = std::make_shared<StaticGate>(object_id6, prefab_id6);
    gate6->setPosition(Eigen::Vector3f(16, -6, 0));
    gate6->setQuaternion(
    Quaternion(0.9239, 0.0, 0.0, 0.3827));
    std::string object_id7 = "unity_gate7";
    std::string prefab_id7 = "aruco_gate2";
    gate7 = std::make_shared<StaticGate>(object_id7, prefab_id7);
    gate7->setPosition(Eigen::Vector3f(14, -8, 0));
    gate7->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 0.0));
    std::string object_id8 = "unity_gate8";
    std::string prefab_id8 = "aruco_gate2";
    gate8 = std::make_shared<StaticGate>(object_id8, prefab_id8);
    gate8->setPosition(Eigen::Vector3f(10, -8, 0));
    gate8->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 0.0));
    std::string object_id9 = "unity_gate9";
    std::string prefab_id9 = "aruco_gate2";
    gate9 = std::make_shared<StaticGate>(object_id9, prefab_id9);
    gate9->setPosition(Eigen::Vector3f(6, -8, 0));
    gate9->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 0.0));*/

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();
  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightmarePX4::~FlightmarePX4() {}

void FlightmarePX4::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  
    //ROS_INFO("pose callback");
  net_updating = false;
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.orientation.z;
  //

  /*if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      // collision happened
      //ROS_INFO("COLLISION");
    }
  }*/
}

void FlightmarePX4::netCallback(std_msgs::Empty sig) {
  net_updating = true;
  std::cout << "entra aqui" << std::endl;
}

void FlightmarePX4::resetCallback(std_msgs::Empty sig) {

    geometry_msgs::PoseArray poses;

    float rng1 = -5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(5-(-5))));
    float rng2 = -5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(5-(-5))));
    gate->setPosition(Eigen::Vector3f(rng1, rng2, 0)); //se aleatoriza sobre el eje y
  
    gate->setQuaternion(
    Quaternion(0.0, 0.0, 0.0, 0.0));
    geometry_msgs::Pose pos;
    pos.position.x = rng1;
    pos.position.y = rng2;
    pos.position.z = 2.0;
    pos.orientation.z = 0.0;
    pos.orientation.w = 0.0;
    //gate_pose_pub.publish(pos);

    /*rng = -5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(5-(-5))));
    gate2->setPosition(Eigen::Vector3f(14, rng, 0)); //se aleatoriza sobre el eje y

    gate2->setQuaternion(
    Quaternion(0.0007963, 0.0, 0.0, 0.9999997));
    geometry_msgs::Pose pos2;
    pos2.position.x = 14;
    pos2.position.y = rng;
    pos2.position.z = 2.0;
    pos2.orientation.z = 0.0007963;
    pos2.orientation.w = -0.9999997;
    //gate_pose_pub.publish(pos);

    rng = -5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(5-(-5))));
    gate3->setPosition(Eigen::Vector3f(18, rng, 0)); //se aleatoriza sobre el eje y

    gate3->setQuaternion(
    Quaternion(-0.9659258, 0.0, 0.0, 0.258819));
    geometry_msgs::Pose pos3;
    pos3.position.x = 18;
    pos3.position.y = rng;
    pos3.position.z = 2.0;
    pos3.orientation.z = -0.9659258;
    pos3.orientation.w = -0.258819;
    //gate_pose_pub.publish(pos);

    rng = -5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(5-(-5))));
    gate4->setPosition(Eigen::Vector3f(22, rng, 0)); //se aleatoriza sobre el eje y

    gate4->setQuaternion(
    Quaternion(0.9238795, 0.0,0.0 ,0.3826834));
    geometry_msgs::Pose pos4;
    pos4.position.x = 22;
    pos4.position.y = rng;
    pos4.position.z = 2.0;
    pos4.orientation.z = 0.9238795;
    pos4.orientation.w = -0.3826834;*/

    poses.poses.push_back(pos);
    /*poses.poses.push_back(pos2);
    poses.poses.push_back(pos3);
    poses.poses.push_back(pos4);*/

    gate_pose_pub.publish(poses);
  
}

void FlightmarePX4::mainLoopCallback(const ros::TimerEvent &event) {
  // empty


  cv::Mat img;
  ros::Time timestamp = ros::Time::now();
  
  //std_srvs::Empty srv;
  //pause_srv_.call(srv);

   
  //br_.sendTransform(tf::StampedTransform(camera_transform_, timestamp, frame_id_, "camera"));
  if (net_updating){
    quad_state_.x[QS::POSX] = 0.0;
    quad_state_.x[QS::POSY] = 0.0;
    quad_state_.x[QS::POSZ] = 0.0;
    quad_state_.x[QS::ATTW] = 0.0;
    quad_state_.x[QS::ATTX] = 0.0;
    quad_state_.x[QS::ATTY] = 0.0;
    quad_state_.x[QS::ATTZ] = 0.0;    
  }
  quad_ptr_->setState(quad_state_);
  
  unity_bridge_ptr_->getRender(0);
  unity_bridge_ptr_->handleOutput();

  std::cout << "aaaaaaaaaaaaa" << std::endl;

  //rgb_camera_->getRGBImage(img);
  //sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  //rgb_msg->header.stamp = timestamp;

  //rgb_pub_.publish(rgb_msg);
  
  if(quad_ptr_->getCollision())
  {
      //ROS_INFO("Collision !!");
  }
  
}

bool FlightmarePX4::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    if (drone_name == "drone1"){
      p_port= "10253";
      s_port= "10254";
    }
    else if (drone_name == "drone2"){
      p_port= "10255";
      s_port= "10256";
    }
    else if (drone_name == "drone3"){
      p_port= "10257";
      s_port= "10258";
    }
    else if (drone_name == "drone4"){
      p_port= "10259";
      s_port= "10260";
    }


    unity_bridge_ptr_ = UnityBridge::getInstance(p_port, s_port);

    unity_bridge_ptr_->addStaticObject(gate);
    /*unity_bridge_ptr_->addStaticObject(gate2);
    unity_bridge_ptr_->addStaticObject(gate3);
    unity_bridge_ptr_->addStaticObject(gate4);
    unity_bridge_ptr_->addStaticObject(gate5);
    unity_bridge_ptr_->addStaticObject(gate6);
    unity_bridge_ptr_->addStaticObject(gate7);
    unity_bridge_ptr_->addStaticObject(gate8);
    unity_bridge_ptr_->addStaticObject(gate9);*/
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);

    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightmarePX4::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(UnityScene::ARUCO);
  return unity_ready_;
}

bool FlightmarePX4::loadParams(void) {
  // load parameters 
  int id;
  pnh_.param("scene_id", id, 1);
  
  pnh_.param("main_loop_freq", main_loop_freq_, 30.0);
  pnh_.param("unity_render", unity_render_, false);
  pnh_.param("frame_id", frame_id_, std::string("base_link"));
  pnh_.param("enable_depth", enable_depth_, false);
  pnh_.param("enable_segmentation", enable_segmentation_, false);
  pnh_.param("enable_opt_flow", enable_opt_flow_, false);
  
  
  return true;
}

}  // namespace flightmare_px4
