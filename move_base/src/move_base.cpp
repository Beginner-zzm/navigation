/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

// MoveBase类，使用actionlib::ActionServer接口，该接口将robot移动到目标位置
  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL), //MoveBaseActionServer* 指针
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),//costmap的实例化指针
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),//nav_core::BaseGlobalPlanner类型的插件
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),//nav_core::BaseLocalPlanner类型的插件
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),//nav_core::RecoveryBehavior类型的插件
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    // 配置参数
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {
//    创建一个action server：MoveBaseActionServer
//    action server接收外部的目标请求，驱动整个路径规划和移动过程
//    actionlib会启动一个线程，当外部请求到来时，调用MoveBase::executeCb回调函数处理
// 创建move_base action,绑定回调函数。定义一个名为move_base的SimpleActionServer。该服务器的Callback为moveBase::executeCb
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
// 触发模式（三种模式：规划、控制、振荡）设置为“规划中”
    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
//  加载参数，从参数服务器获取一些参数，包括两个规划器名称、代价地图坐标系、规划频率、控制周期等
    std::string global_planner, local_planner;
    // turtlebot 默认 base_global_planner 为 navfn/NavfnROS，默认 base_local_planner 为 dwa_local_planner/DWAPlannerROS
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //set up plan triple buffer
//  为三种规划器设置内存缓冲区（planner_plan_保存最新规划的路径，传递给latest_plan
//  然后latest_plan_通过executeCycle中传给controller_plan_）
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
// 创建并启动global planner线程，此线程负责全局规划器的路径选择过程
// 设置规划器线程，planner_thread_是boost::thread*类型的指针
// 其中，MoveBase::planThread()函数是planner线程的入口。这个函数需要等待actionlib服务器的cbMoveBase::executeCb来唤醒启动。
// 主要作用是调用全局路径规划获取路径，同时保证规划的周期性以及规划超时清除goal
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for commanding the base
//    订阅和发布相关的topic
//    发布/cmd_vel
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
//    发布 /move_base/current_goal
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
//    发布 /move_base/goal
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    // 提供消息类型为geometry_msgs::PoseStamped的发送goals的接口，比如cb为MoveBase::goalCB，在rviz中输入的目标点就是通过这个函数来响应的
    ros::NodeHandle simple_nh("move_base_simple");
//    订阅 /move_base_simple/goal
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    // 设置costmap参数，技巧是把膨胀层设置为大于机器人的半径
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
// 设置代价地图指针，planner_costmap_ros_是costmap_2d::Costmap2DROS*类型的实例化指针
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
//    先暂停运行
    planner_costmap_ros_->pause();

//    costmap的动态库位于/opt/ros/melodic/lib/libcostmap_2d.so，会启动一个线程处理工作流程


    //initialize the global planner    
    try {  //planner_是全局规划器的实例对象，是boost::shared_ptr<nav_core::BaseGlobalPlanner>类型
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
//    设置局部代价地图指针
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner    
    try {   //tc_是局部规划器的实例对象
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
//    启动global和local costmap的处理, 开始更新costmap
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
//    /move_base/make_plan 全局规划
// MoveBase::planService（）函数写了全局规划的策略，以多少距离向外搜索路径
// 该服务的请求为一个目标点，响应为规划的轨迹，但不执行该轨迹。
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
//    /move_base/clear_costmaps  开始清除地图服务
// 调用了MoveBase::clearCostmapsService（）函数，提供清除一次costmap的功能
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    // 如果不小心关闭了costmap， 则停用
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
//    加载recovery behavior,加载指定的恢复器，加载不出来则使用默认的，这里包括了找不到路自转360
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

//    initially, we'll need to make a plan
//    设置move_base的状态为PLANNING
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
//    执行recovery behavior
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
//  10.启动action server(move_base动作器)
    as_->start();

//-------------------------------------------------------------
//    此时move_base 可以处理外部的导航请求了
//-------------------------------------------------------------


//    启动动态配置参数功能，move_base的一些参数支持动态修改，通过此功能实现
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
//  回调函数MoveBase::reconfigureCB（），配置了各种参数
//    至此整个move_base导航启动完成

  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }

// MoveBase::goalCB（）函数，传入的是goal，主要作用是为rviz等提供调用借口
// 将geometry_msgs::PoseStamped形式的goal转换成move_base_msgs::MoveBaseActionGoal，再发布到对应类型的goal话题中
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

// 调用了MoveBase::clearCostmapsService（）函数，提供清除一次costmap的功能
  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
    // 值得注意的是，其中有一个函数resetLayers()，调用的是costmap包（注意！！外部链接！！！）
    // 该函数的功能是重置地图，内部包括重置总地图、重置地图各层
  }

// MoveBase::planService（）函数写了全局规划的策略，以多少距离向外搜索路径
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
// 获取起始点，如果没有起始点，那就获取当前的全局位置为起始点
    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
       //如果起始点为空，设置global_pose为起始点
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }
// 制定规划策略：
    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      //在规定的公差范围内向外寻找可行的goal
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;  ////以3倍分辨率的增量向外寻找
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
             //不在本位置的外侧layer查找，太近的不找
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            //从两个方向x、y查找精确的goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              //第一次遍历如果偏移量过小,则去除这个点或者上一点
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    // 然后把规划后的global_plan附给resp，并且传出去
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

// 析构函数，释放了内存
  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }
// 在planThread中调用 makePlan(temp_goal, *planner_plan_)
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

// 发布零速
  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

// 主要用于检查四元数的合法性
  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    // 1、首先检查四元数是否元素完整
    // isfinite()函数是cmath标头的库函数，用于检查给定值是否为有限值
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    // 2、检查四元数是否趋近于0
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    // 3、对四元数规范化，转化为vector
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  // MoveBase::wakePlanner（）函数中用planner_cond_开启了路径规划的线程
  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }
// 全局路径规划线程
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    //1. 创建递归锁
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      //2.判断是否阻塞线程
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        //当 std::condition_variable 对象的某个 wait 函数被调用的时候，
      //它使用 std::unique_lock(通过 std::mutex) 来锁住当前线程。
      //当前线程会一直被阻塞，直到另外一个线程在相同的 std::condition_variable 对象上调用了 notification 函数来唤醒当前线程。
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
       //3. 获取规划的全局路径
      //这里的makePlan作用是获取机器人的位姿作为起点，然后调用全局规划器的makePlan返回规划路径，存储在planner_plan_
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){  //4.如果获得了plan,则将其赋值给latest_plan_
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        // 将planner_plan_与latest_plan_互换
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      //5. 达到一定条件后停止路径规划,进入清障模式
      //如果是则进入自转模式，否则应该会等待MoveBase::executeCycle的唤醒再次规划
      else if(state_==PLANNING){ //如果没有规划出路径，并且处于PLANNING状态，则判断是否超过最大规划周期或者规划次数
      //仅在MoveBase::executeCb及其调用的MoveBase::executeCycle或者重置状态时会被设置为PLANNING
      //一般是刚获得新目标，或者得到路径但计算不出下一步控制时重新进行路径规划
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      //6.设置睡眠模式
      //如果还没到规划周期则定时器睡眠，在定时器中断中通过planner_cond_唤醒，这里规划周期为0
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

// move_base action的回调函数。创建move_base类时定义一个名为move_base的SimpleActionServer，该服务器的Callback为moveBase::executeCb
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    // 如果目标非法，则直接返回
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
// 将目标的坐标系统一转换为全局坐标系
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    publishZeroVelocity();
    //we have a goal so start the planner
    // 设置目标点并唤醒路径规划线程
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();
// 发布goal，设置控制频率
    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    // 开启costmap更新
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    // 重置时间标志位
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    // 开启循环，循环判断是否有新的goal抢占（重要！！！）
    while(n.ok())
    {
       //7. 修改循环频率
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
//8. 如果获得一个抢占式目标
      if(as_->isPreemptRequested()){  //action的抢断函数
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          // 如果有新的目标，会接受，但不会关闭其他进程
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
  //9.如果目标无效,则返回
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
//10.将目标转换为全局坐标系
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          //11.设置状态为PLANNING
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          //12. 设置目标点并唤醒路径规划线程
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          //13. 把goal发布给可视化工具
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          //14. 重置规划时间
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          //14.重置状态,设置为抢占式任务
          resetState();

          //notify the ActionServer that we've successfully preempted
          //通知ActionServer已成功抢占
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      //15.如果目标点的坐标系和全局地图的坐标系不相同
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        //16,转换目标点坐标系
        goal = goalToGlobalFrame(goal);  

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        //17. 设置目标点并唤醒路径规划线程
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        //18.重置规划器相关时间标志位
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      //19. 到达目标点的真正工作，控制机器人进行跟随
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      //20.如果完成任务则返回
      if(done){
        return;
      }

// 其中，done的值即为MoveBase::executeCycle（）函数的返回值，这个值非常重要，直接判断了是否到达目标点；
// MoveBase::executeCycle（）函数是控制机器人进行跟随的函数（重要！！！）


      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
      //21. 执行休眠动作
      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }
// 剩下的就是解放线程，反馈给action结果
    //wake up the planner thread so that it can exit cleanly
    //22. 唤醒计划线程，以便它可以干净地退出
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
     //23. 如果节点结束就终止并返回
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

// 求两个坐标点之间的直线距离
// hypot()函数是cmath标头的库函数，用于查找给定数字的斜边，接受两个数字并返回斜边的计算结果，即sqrt(x * x + y * y)
// hypot()函数语法：hypot(x, y);
  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

// 控制机器人进行跟随（重要！！！！）该函数的两个参数分别是目标点位姿以及规划出的全局路径
// 通过上述两个已知，利用局部路径规划器直接输出轮子速度，控制机器人按照路径走到目标点，成功返回真，否则返回假
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    //变量定义
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

// 获取机器人当前位置
    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //push the feedback out    发布给server的feedback
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    // 判断当前位置和是否振荡，其中distance函数返回的是两个位置的直线距离（欧氏距离）
    // recovery_trigger_是枚举RecoveryTrigger的对象
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index
      //如果上次的恢复是由振荡引起的，重置恢复指数
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    // 地图数据超时，即观测传感器数据不够新，停止机器人，返回false
    // 其中publishZeroVelocity（）函数把geometry_msgs::Twist类型的cmd_vel设置为0并发布出去:
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    // 如果获取新的全局路径,则将它传输给控制器，完成latest_plan_到controller_plan_的转换（提示：头文件那里讲过规划转换的规则）
    if(new_global_plan_){  //该变量在规划器线程中，当新的路径被规划出来，该值被置1
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      // 将controller_plan_与latest_plan_互换
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");
 //5. 将全局路径设置到局部路径规划器中
//  其中，tc_是局部规划器的指针，setPlan是TrajectoryPlannerROS的函数（注意！！！跟外部包有关系了！！）
      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        //同时也关闭规划器线程，没必要规划了
        lock.lock();
        runPlanner_ = false;
        lock.unlock();
//6.设置动作中断,返回true
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      //如果全局路径有效，则不需要recovery
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

// 然后判断move_base状态，一般默认状态或者接收到一个有效goal时是PLANNING，在规划出全局路径后state_会由PLANNING变为CONTROLLING
// 如果规划失败则由PLANNING变为CLEARING

    //the move_base state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:  //机器人规划状态，尝试获取一条全局路径
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;   
          planner_cond_.notify_one();     //还在PLANNING中则唤醒规划线程让它干活
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        // 如果到达目标点，重置状态，设置动作成功，返回true，其中isGoalReached()函数是TrajectoryPlannerROS的函数（注意！！！这里跟外部包有关！！）
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();
          //重置状态，关闭规划器线程，设置告知Client结果
          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }
// 如果超过震荡时间，停止机器人，设置清障标志位
//last_oscillation_reset_获得新目标会重置,距离超过震荡距离（默认0.5）会重置，进行recovery后会重置
//所以是太久没有发生上面的事就震动一下，防止长时间在同一个地方徘徊？？？？这里oscillation_timeout_默认为0 ，不发生。
        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }

        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
// 获取有效速度,如果获取成功，直接发布到cmd_vel
        if(tc_->computeVelocityCommands(cmd_vel)){  //如果局部路径规划成功
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);  //发布控制速度信息
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {  //局部规划失败
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          //如果没有获取有效速度，则判断是否超过尝试时间，如果超时，则停止机器人，进入清障模式：
          if(ros::Time::now() > attempt_end){  //判断是否控制超时
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{//如果没有超时，则再全局规划一个新的路径：
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
             //没超时则启动规划器线程重新规划   
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case CLEARING://三种原因需要recovery：机器人清障状态，规划或者控制失败，恢复或者进入到清障模式 
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        // 如果有可用恢复器，执行恢复动作，并设置状态为PLANNING
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){ //遍历recovery方法
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          recovery_status_pub_.publish(msg);

          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{//如果没有可用恢复器，结束动作，返回true //遍历完还是不行 
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

//        分原因发布消息
          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          //已经done了
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      // clear_costmap_recovery将代价地图恢复到静态地图的样子
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      // rotate_recovery让机器人原地360°旋转
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_){
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

// 重置move_base action的状态，设置速度为0
  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

  // 全局坐标时间戳是否在costmap要求下
    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
