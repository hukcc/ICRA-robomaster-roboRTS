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

namespace roborts_global_planner
{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;
using roborts_common::NodeState;
GlobalPlannerNode::GlobalPlannerNode() : //这里不是类的派生！！！！如果是那必须是要写类型名的 这边冒号后面的是对于对象的成员变量赋值！！！
                                         new_path_(false), pause_(false), node_state_(NodeState::IDLE), error_info_(ErrorCode::OK),
                                         as_(nh_, "global_planner_node_action", boost::bind(&GlobalPlannerNode::GoalCallback, this, _1), false)
{
  //这一系列被初始化的变量分别是
  //new_path_:一个标志位，判断是否出现了新的计划路径
  //pause_:判断是否是global生成的合适路径 如果是就暂停路径规划？
  //node_state_:全局路径规划节点自身的状态
  //error_info_:存储错误信息？ 或者是判断是否错误的flag
  //as_:ros 中对应的AC（运动）模块对于运动的控制。

  if (Init().IsOK())
  {                                                       // 这里应该是判断是否初始化完成  对 这里的IsOK是检查Init的返回值 相当于 ErrorInfo.IsOK()
    ROS_INFO("Global planner initialization completed."); //记录初始化完成
    StartPlanning();                                      //这一步是对路径进行规划
    as_.start();                                          //对上面得到的路径去做实际的动作      跳转2。8好了
  }
  else
  { //否则报错并将节点当前的状态设置为failure。
    ROS_ERROR("Initialization failed.");
    SetNodeState(NodeState::FAILURE);
  }
}

ErrorInfo GlobalPlannerNode::Init()
{ //初始化全局规划器

  // Load proto planning configuration parameters
  GlobalPlannerConfig global_planner_config;                                                                                   //实例化一个用来加载参数的对象 从proto文件里加载参数
  std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/config/global_planner_config.prototxt"; //声明一个字符串变量来储存目标文件的路径
  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),                                                                //c_str()是用来将string类转化为c字符串的 因为io接口是c语言
                                             &global_planner_config))   //从文件内容可知 调用的是A*算法
  { //这里gpc是用来接受参数的     这里返回的是是否成功读取文件中的内容
    ROS_ERROR("Cannot load global planner protobuf configuration file.");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "Cannot load global planner protobuf configuration file."); //如果失败就报错跳出 此时ISOK函数就会检测返回值
  }
  //如果成功加载 就将配置文件中的各种参数分配下去

  selected_algorithm_ = global_planner_config.selected_algorithm();                            //选择算法
  cycle_duration_ = std::chrono::microseconds((int)(1e6 / global_planner_config.frequency())); //循环持续时间
  max_retries_ = global_planner_config.max_retries();                                          //最大尝试次数
  goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();                  //目标距离公差（大概是误差把）
  goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();                        //目标角度公差

  // ROS path visualize         可视化路径？ 应该是用于调试的
  ros::NodeHandle viz_nh("~");
  path_pub_ = viz_nh.advertise<nav_msgs::Path>("path", 10);

  // Create tf listener       用于监听发布到tf的消息 对应的有发布器
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10)); //这里是创建了一个智能指针 每10ms监听一次消息？

  // Create global costmap
  std::string map_path = ros::package::getPath("roborts_costmap") + \ //声明字符串保存代价地图的路径
                         "/config/costmap_parameter_config_for_global_plan.prototxt";
  costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>("global_costmap",
                                                                     *tf_ptr_,
                                                                     map_path.c_str()); //这里对这个关键！！！的指针赋值 实际上用的也是一个监听 不断监听最新地图的保存路径 路径保存给map_path 内容可以通过costmap_ptr_来调用
  // Create the instance of the selected algorithm    //实例化算法
  global_planner_ptr_ = roborts_common::AlgorithmFactory<GlobalPlannerBase, CostmapPtr>::CreateAlgorithm( //传入算法的名称 以及对应的参数文件信息
      selected_algorithm_, costmap_ptr_);   //返回对应的可用算法  如果在哈希表里面没有查找到对应的算法就会返回空指针
  if (global_planner_ptr_ == nullptr)
  {
    ROS_ERROR("global planner algorithm instance can't be loaded");           //如果没有查找到对应的算法就报错退出 并标记初始化失败 原因等
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "global planner algorithm instance can't be loaded");
  }

  // Initialize path frame from global costmap    从全局地图初始化路径地图
  path_.header.frame_id = costmap_ptr_->GetGlobalFrameID();       //返回值是一个string 可以说是返回了地图的名字
  return ErrorInfo(ErrorCode::OK);    //标记初始化完成
}

void GlobalPlannerNode::GoalCallback(const roborts_msgs::GlobalPlannerGoal::ConstPtr &msg)
{

  ROS_INFO("Received a Goal from client!");

  //Update current error and info
  ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  //Set the current goal
  SetGoal(msg->goal);

  //If the last state is not running, set it to running
  if (GetNodeState() != NodeState::RUNNING)
  {
    SetNodeState(NodeState::RUNNING);
  }

  //Notify the condition variable to stop lock waiting the fixed duration
  {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.notify_one();
  }

  while (ros::ok())
  {
    // Preempted and Canceled
    if (as_.isPreemptRequested())
    {
      if (as_.isNewGoalAvailable())
      {
        as_.setPreempted();
        ROS_INFO("Override!");
        break;
      }
      else
      {
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
    if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS || node_state == NodeState::FAILURE)
    {
      roborts_msgs::GlobalPlannerFeedback feedback;
      roborts_msgs::GlobalPlannerResult result;
      // If error occurs or planner produce new path, publish the feedback
      if (!error_info.IsOK() || new_path_)
      {
        if (!error_info.IsOK())
        {
          feedback.error_code = error_info.error_code();
          feedback.error_msg = error_info.error_msg();
          SetErrorInfo(ErrorInfo::OK());
        }
        if (new_path_)
        {
          feedback.path = path_;
          new_path_ = false;
        }
        as_.publishFeedback(feedback);
      }

      // After get the result, deal with actionlib server and jump out of the loop
      if (node_state == NodeState::SUCCESS)
      {
        result.error_code = error_info.error_code();
        as_.setSucceeded(result, error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
      else if (node_state == NodeState::FAILURE)
      {
        result.error_code = error_info.error_code();
        as_.setAborted(result, error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

NodeState GlobalPlannerNode::GetNodeState()
{
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  return node_state_;
}

void GlobalPlannerNode::SetNodeState(NodeState node_state)
{
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  node_state_ = node_state;
}

ErrorInfo GlobalPlannerNode::GetErrorInfo()
{
  std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
  return error_info_;
}

void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info)
{
  std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
  error_info_ = error_info;
}

geometry_msgs::PoseStamped GlobalPlannerNode::GetGoal()
{                                                   //这里由于是多线程协作 因此互斥机制就非常重要了
  std::lock_guard<std::mutex> goal_lock(goal_mtx_); //上锁 获取这个对象然后返回目标点 lock_guard其实只是将上锁和解锁和为一体 并没有其他特殊功能
  return goal_;
}

void GlobalPlannerNode::SetGoal(geometry_msgs::PoseStamped goal)
{
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  goal_ = goal;
}

void GlobalPlannerNode::StartPlanning()
{
  SetNodeState(NodeState::IDLE);                                    //先改变当前节点的状态标记为空闲 emmmmm为啥是空闲 只是声明了线程还没有调用运行？
  plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this); //单独开一个线程去规划路径
}

void GlobalPlannerNode::StopPlanning()
{
  SetNodeState(NodeState::RUNNING);
  if (plan_thread_.joinable())
  {
    plan_thread_.join();
  }
}

void GlobalPlannerNode::PlanThread()
{                                                                      //在单独的线程中进行路径规划
  ROS_INFO("Plan thread start!");                                      //终端消息
  geometry_msgs::PoseStamped current_start;                            //这里是某一种消息的类型 实际上就是点（参考ros-wiki） point-stamped：点-邮戳所以是用来标记点的数据类型
  geometry_msgs::PoseStamped current_goal;                             //这里分别声明起点和终点的实例化对象
  std::vector<geometry_msgs::PoseStamped> current_path;                //用点集来表示路径
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0); //应该是设置的线程休眠时间 这里采用的是标准的秒。。。
  ErrorInfo error_info;                                                //声明错误信息存储的实例化对象  内容默认是无错误
  int retries = 0;                                                     //标记重复次数
  while (ros::ok())
  {                                                      //这里的ros：：ok函数其实是对于内核信号的一个监听 如果没有收到中断信号则一直保持ture
    ROS_INFO("Wait to plan!");                           //终端消息
    std::unique_lock<std::mutex> plan_lock(plan_mutex_); //声明一个锁的模板管理 plan_lock对象以独占的方式对planmutex进行上所和解锁 在这个模板类的生命周期内原来的对象将会一直保持上锁的状态 当模板对象消亡的时候就会进行解锁
    //简单来说 该线程独占plan_mutex_对象 所以暂时还不清楚这个对象是干啥用的 这些对象的成员和构造函数应该是给我们自己去写的。。。
    plan_condition_.wait_for(plan_lock, sleep_time); //这里是一个条件变量对象 当该对象的一个wait函数被调用 就会对其控制的进程进行阻塞直到另外一个线程在相同的 std::condition_variable 对象上调用了 notification 函数来唤醒当前线程。
                                                     //对于这里的wait_for函数而言其实就是阻塞这个进程sleep_time的时长
    while (GetNodeState() != NodeState::RUNNING)
    { //从一个互斥锁的对象那里获取某个节点的状态 如果是运行的 就让该线程休眠 循环读取相当于在等某一个线程运行完

      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    ROS_INFO("Go on planning!"); //上面这里可以理解为 先等一下看看别的进程是否运行完了如果运行完了就继续往下走

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now(); //计时函数的起点时间

    {                                                                                                        //这是costmapinterface的一个智能指针的对象
      std::unique_lock<roborts_costmap::Costmap2D::mutex_t> lock(*(costmap_ptr_->GetCostMap()->GetMutex())); //这里是锁了一个 路径？ 占用这个变量拿来规划？
                                                                                                             //获取代价地图
      bool error_set = false;                                                                                //位置标志位
      //Get the robot current pose
      //估计是指向建好的图的指针costmap_ptr_
      while (!costmap_ptr_->GetRobotPose(current_start))
      { //如果没有获取当前的全局位置则进入循环 如果成功就把这个位置作为开始位置
        if (!error_set)
        {
          ROS_ERROR("Get Robot Pose Error.");
          SetErrorInfo(ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error.")); //如果当前循环中获取姿态错误则将标志为标记为错误
          error_set = true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1)); //循环休眠1s
      }

      //Get the robot current goal and transform to the global frame
      current_goal = GetGoal(); //估计是从别的线程处获取全局目标位置 把goal互斥对象放进lock_guard然后返回goal

      if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID())
      {                                                              //如果目标点的位置和全局地图中的不同？
        current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal); //貌似只是为了发布信息？
        SetGoal(current_goal);                                       //应该是用来同步目标位置和它在地图上的标记？
      }

      //Plan
      error_info = global_planner_ptr_->Plan(current_start, current_goal, current_path); //调用路径规划算法 的规划函数   上面的部分只是确定一组合适的起点和终点 放入到这个plan函数之后得到了规划好的路径 存放在current_path

    } //以上这一块就是global_planner的主要内容

    if (error_info.IsOK())    //如果成功规划了路径
    {
      //When planner succeed, reset the retry times
      retries = 0;    //规划成功重置尝试次数
      PathVisualization(current_path);  //路径可视化？

      //Set the goal to avoid the same goal from getting transformed every time
      current_goal = current_path.back();
      SetGoal(current_goal);                //将路径规划的终点设置为当前的终点（因为在规划的时候会有误差搜索所以终点有可能改变 如果这里不这么做就有可能再次触发规划）

      //Decide whether robot reaches the goal according to tolerance    //由误差决定是否要走过去 误差？
      if (GetDistance(current_start, current_goal) < goal_distance_tolerance_ && GetAngle(current_start, current_goal) < goal_angle_tolerance_)
      {
        SetNodeState(NodeState::SUCCESS);       //如果距离和角度都在误差允许范围内 就标志规划成功
      }
    }
    else if (max_retries_ > 0 && retries > max_retries_)        //如果超出了重试的上限
    {
      //When plan failed to max retries, return failure
      ROS_ERROR("Can not get plan with max retries( %d )", max_retries_);       //终端显示尝试的次数
      error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max retries.");   //报错
      SetNodeState(NodeState::FAILURE);     //将规划状态标记为失败
      retries = 0;      //尝试次数清0
    }
    else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR))     //无法到达的目标    错误的处理方式和上面差不多
    {
      //When goal is not reachable, return failure immediately
      ROS_ERROR("Current goal is not valid!");
      SetNodeState(NodeState::FAILURE);
      retries = 0;
    }
    else
    {
      //Increase retries
      retries++;
      ROS_ERROR("Can not get plan for once. %s", error_info.error_msg().c_str());       //一次规划到达不了的点。。。 有点懵
    }
    // Set and update the error info    更新（错误）状态信息
    SetErrorInfo(error_info);

    // Deal with the duration to wait       处理其他线程？
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    sleep_time = cycle_duration_ - execution_duration;

    // Report warning while planning timeout    如果这个线程超时就报错
    if (sleep_time <= std::chrono::microseconds(0))
    {
      ROS_ERROR("The time planning once is %ld beyond the expected time %ld",
                execution_duration.count(),
                cycle_duration_.count());
      sleep_time = std::chrono::microseconds(0);
      SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }
  }

  ROS_INFO("Plan thread terminated!");
}

void GlobalPlannerNode::PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path)
{
  path_.poses = path;   //？待查  ？  反正是把路径赋值给一个消息？
  path_pub_.publish(path_); //发布路径消息  方便某个什么东西来把路径可视化
  new_path_ = true;     //标志找到了新的路径
}

double GlobalPlannerNode::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                      const geometry_msgs::PoseStamped &pose2)      //直接获取起点终点之间的直线距离
{
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GlobalPlannerNode::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                   const geometry_msgs::PoseStamped &pose2)     //直接获取起点终点之间的角度 （用于判断误差搜索终点的时候变更的终点是否可以接受
{
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
}

GlobalPlannerNode::~GlobalPlannerNode()
{
  StopPlanning();
}

} //namespace roborts_global_planner

int main(int argc, char **argv)
{

  ros::init(argc, argv, "global_planner_node");             //ros的初始化 这是再调用任何ros中的函数前必须的一步。 可以理解为头文件？ 第三个参数必须是节点的名字 这里保持与文件名一致
  roborts_global_planner::GlobalPlannerNode global_planner; //初始化一个全局规划对象 这边开始运行就包含在对象的构造函数中了
  ros::spin();                                              //用于响应topic？ 在ros中各个节点的消息不会被立刻处理而是会派到消息队列中当调用spin这个轮转函数之后才会进行一轮处理 在这个轮转函数之后 main函数中的所有回调函数都会一直不断的被调用
  return 0;
}
