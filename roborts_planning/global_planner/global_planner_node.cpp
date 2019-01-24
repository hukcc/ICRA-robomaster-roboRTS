/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <csignal>

#include "global_planner_node.h"

namespace roborts_global_planner{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;
using roborts_common::NodeState;
GlobalPlannerNode::GlobalPlannerNode() :  //这里不是类的派生！！！！如果是那必须是要写类型名的 这边冒号后面的是对于对象的成员变量赋值！！！
    new_path_(false),pause_(false), node_state_(NodeState::IDLE), error_info_(ErrorCode::OK),
    as_(nh_,"global_planner_node_action",boost::bind(&GlobalPlannerNode::GoalCallback,this,_1),false) {
//这一系列被初始化的变量分别是
    //new_path_:一个标志位，判断是否出现了新的计划路径
    //pause_:判断是否是global生成的合适路径 如果是就暂停路径规划？
    //node_state_:全局路径规划节点自身的状态
    //error_info_:存储错误信息？ 或者是判断是否错误的flag
    //as_:ros 中对应的AC（运动）模块对于运动的控制。

  if (Init().IsOK()) {  //检查error_code是否为OK 如果是则开始路径规划
    ROS_INFO("Global planner initialization completed.");//记录初始化完成
    StartPlanning();  //这一步是对路径进行规划
    as_.start();  //对上面得到的路径去做实际的动作
  } else {  //否则报错并将节点当前的状态设置为failure。
    ROS_ERROR("Initialization failed.");
    SetNodeState(NodeState::FAILURE);
  }

}

ErrorInfo GlobalPlannerNode::Init() {

  // Load proto planning configuration parameters
  GlobalPlannerConfig global_planner_config;
  std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/config/global_planner_config.prototxt";
  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &global_planner_config)) {
    ROS_ERROR("Cannot load global planner protobuf configuration file.");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "Cannot load global planner protobuf configuration file.");
  }


  selected_algorithm_ = global_planner_config.selected_algorithm();
  cycle_duration_ = std::chrono::microseconds((int) (1e6 / global_planner_config.frequency()));
  max_retries_ = global_planner_config.max_retries();
  goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();
  goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();

  // ROS path visualize
  ros::NodeHandle viz_nh("~");
  path_pub_ = viz_nh.advertise<nav_msgs::Path>("path", 10);

  // Create tf listener
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  // Create global costmap
  std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_global_plan.prototxt";
  costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>("global_costmap",
                                                                           *tf_ptr_,
                                                                           map_path.c_str());
  // Create the instance of the selected algorithm
  global_planner_ptr_ = roborts_common::AlgorithmFactory<GlobalPlannerBase,CostmapPtr >::CreateAlgorithm(
      selected_algorithm_, costmap_ptr_);
  if (global_planner_ptr_== nullptr) {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "global planner algorithm instance can't be loaded");
  }


  // Initialize path frame from global costmap
  path_.header.frame_id = costmap_ptr_->GetGlobalFrameID();
  return ErrorInfo(ErrorCode::OK);
}

void GlobalPlannerNode::GoalCallback(const roborts_msgs::GlobalPlannerGoal::ConstPtr &msg) {

  ROS_INFO("Received a Goal from client!");

  //Update current error and info
  ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  //Set the current goal
  SetGoal(msg->goal);

  //If the last state is not running, set it to running
  if (GetNodeState() != NodeState::RUNNING) {
    SetNodeState(NodeState::RUNNING);
  }

  //Notify the condition variable to stop lock waiting the fixed duration
  {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.notify_one();
  }



  while (ros::ok()) {
    // Preempted and Canceled
    if (as_.isPreemptRequested()) {
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        ROS_INFO("Override!");
        break;
      }else{
        as_.setPreempted();
        SetNodeState(NodeState::IDLE);
        ROS_INFO("Cancel!");
        break;
      }
    }

    // Update the current state and error info
    node_state = GetNodeState();
    error_info = GetErrorInfo();
    //TODO: seem useless to check state here, as it would never be IDLE state
    if(node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS || node_state == NodeState::FAILURE) {
      roborts_msgs::GlobalPlannerFeedback feedback;
      roborts_msgs::GlobalPlannerResult result;
      // If error occurs or planner produce new path, publish the feedback
      if (!error_info.IsOK() || new_path_) {
        if (!error_info.IsOK()) {
          feedback.error_code = error_info.error_code();
          feedback.error_msg = error_info.error_msg();
          SetErrorInfo(ErrorInfo::OK());
        }
        if (new_path_) {
          feedback.path = path_;
          new_path_ = false;
        }
        as_.publishFeedback(feedback);
      }

      // After get the result, deal with actionlib server and jump out of the loop
      if(node_state == NodeState::SUCCESS){
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
      else if(node_state == NodeState::FAILURE){
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }

}

NodeState GlobalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  return node_state_;
}

void GlobalPlannerNode::SetNodeState(NodeState node_state) {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  node_state_ = node_state;
}

ErrorInfo GlobalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
  return error_info_;
}

void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info) {
  std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
  error_info_ = error_info;
}

geometry_msgs::PoseStamped GlobalPlannerNode::GetGoal() {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  return goal_;
}

void GlobalPlannerNode::SetGoal(geometry_msgs::PoseStamped goal) {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  goal_ = goal;
}

void GlobalPlannerNode::StartPlanning() {
  SetNodeState(NodeState::IDLE);  //先改变当前节点的状态标记
  plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this);  //单独开一个线程去规划路径
}

void GlobalPlannerNode::StopPlanning() {
  SetNodeState(NodeState::RUNNING);
  if (plan_thread_.joinable()) {
    plan_thread_.join();
  }
}

void GlobalPlannerNode::PlanThread() {  //在单独的线程中进行路径规划
  ROS_INFO("Plan thread start!"); //终端消息
  geometry_msgs::PoseStamped current_start;   //这里是某一种消息的类型 实际上就是点（参考ros-wiki）
  geometry_msgs::PoseStamped current_goal;  //这里分别声明起点和终点的实例化对象
  std::vector<geometry_msgs::PoseStamped> current_path;   //用点集来表示路径
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);  //应该是设置的线程休眠时间 这里采用的是标准的秒。。。
  ErrorInfo error_info;   //声明错误信息存储的实例化对象  内容默认是无错误
  int retries = 0;  //未知 重试？ 大概是标记重复次数
  while (ros::ok()) {   //这里的ros：：ok函数其实是对于内核信号的一个监听 如果没有收到中断信号则一直保持ture
    ROS_INFO("Wait to plan!");  //终端消息
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);    
    plan_condition_.wait_for(plan_lock, sleep_time);
    while (GetNodeState()!=NodeState::RUNNING){
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    ROS_INFO("Go on planning!");

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    {
      std::unique_lock<roborts_costmap::Costmap2D::mutex_t> lock(*(costmap_ptr_->GetCostMap()->GetMutex()));
      bool error_set = false;
      //Get the robot current pose
      while (!costmap_ptr_->GetRobotPose(current_start)) {
        if (!error_set) {
          ROS_ERROR("Get Robot Pose Error.");
          SetErrorInfo(ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error."));
          error_set = true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }

      //Get the robot current goal and transform to the global frame
      current_goal = GetGoal();

      if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID()) {
        current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
        SetGoal(current_goal);
      }

      //Plan
      error_info = global_planner_ptr_->Plan(current_start, current_goal, current_path);

    }

    if (error_info.IsOK()) {
      //When planner succeed, reset the retry times
      retries = 0;
      PathVisualization(current_path);

      //Set the goal to avoid the same goal from getting transformed every time
      current_goal = current_path.back();
      SetGoal(current_goal);

      //Decide whether robot reaches the goal according to tolerance
      if (GetDistance(current_start, current_goal) < goal_distance_tolerance_
          && GetAngle(current_start, current_goal) < goal_angle_tolerance_
          ) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (max_retries_ > 0 && retries > max_retries_) {
      //When plan failed to max retries, return failure
      ROS_ERROR("Can not get plan with max retries( %d )", max_retries_ );
      error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max retries.");
      SetNodeState(NodeState::FAILURE);
      retries=0;
    } else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR)){
      //When goal is not reachable, return failure immediately
      ROS_ERROR("Current goal is not valid!");
      SetNodeState(NodeState::FAILURE);
      retries=0;
    }
    else {
      //Increase retries
      retries++;
      ROS_ERROR("Can not get plan for once. %s", error_info.error_msg().c_str());
    }
    // Set and update the error info
    SetErrorInfo(error_info);

    // Deal with the duration to wait
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    sleep_time = cycle_duration_ - execution_duration;

    // Report warning while planning timeout
    if (sleep_time <= std::chrono::microseconds(0)) {
      ROS_ERROR("The time planning once is %ld beyond the expected time %ld",
                execution_duration.count(),
                cycle_duration_.count());
      sleep_time = std::chrono::microseconds(0);
      SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }
  }


  ROS_INFO("Plan thread terminated!");
}

void GlobalPlannerNode::PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path) {
  path_.poses = path;
  path_pub_.publish(path_);
  new_path_ = true;
}

double GlobalPlannerNode::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                      const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GlobalPlannerNode::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                   const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
}

GlobalPlannerNode::~GlobalPlannerNode() {
  StopPlanning();
}

} //namespace roborts_global_planner

int main(int argc, char **argv) {

  ros::init(argc, argv, "global_planner_node"); //ros的初始化 这是再调用任何ros中的函数前必须的一步。 可以理解为头文件？ 第三个参数必须是节点的名字 这里保持与文件名一致
  roborts_global_planner::GlobalPlannerNode global_planner; //初始化一个全局规划对象 这边开始运行就包含在对象的构造函数中了
  ros::spin();  //用于响应topic？ 在ros中各个节点的消息不会被立刻处理而是会派到消息队列中当调用spin这个轮转函数之后才会进行一轮处理 在这个轮转函数之后 main函数中的所有回调函数都会一直不断的被调用
  return 0;
}
