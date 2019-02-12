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

#include "local_planner/local_planner_node.h"

namespace roborts_local_planner {

using roborts_common::NodeState;
LocalPlannerNode::LocalPlannerNode() :    //实例化之后转到构造函数  同样是一开始先对重要变量初始化（声明赋值）
    local_planner_nh_("~"),   //emmm这个我记得是句柄  一般都是None这里不知道是什么操作
    as_(local_planner_nh_, "/local_planner_node_action", boost::bind(&LocalPlannerNode::ExcuteCB, this, _1), false),    //as_初始化操作执行服务端
    initialized_(false), node_state_(roborts_common::NodeState::IDLE),  //initialize标志是否初始化完成？  将节点状态初始化为空闲
    node_error_info_(roborts_common::ErrorCode::OK), max_error_(5),     //初始化节点错误信息
    local_cost_(nullptr), tf_(nullptr) {                                //初始化两个分别指向局部规划地图 和坐标转化的指针
  if (Init().IsOK()) {    //做一下初始化  完成后检查是否正常的完成
    ROS_INFO("local planner initialize completed.");    //
  } else {
    ROS_WARN("local planner initialize failed.");
    SetNodeState(NodeState::FAILURE);
  }                                                   //与gp明显不是同一个人写的。。。  初始化操作大致相同 但是对于gp来说路径规划是单独的一步 这边可能是把他整合到行动服务端中执行了
  as_.start();    //初始化完成之后  进入行动服务器 根据初始化规则跳转&LocalPlannerNode::ExcuteCB
}

LocalPlannerNode::~LocalPlannerNode() {
  StopPlanning();
}

roborts_common::ErrorInfo LocalPlannerNode::Init() {    //局部规划节点的初始化函数
  ROS_INFO("local planner start");    //终端消息  开始局部规划
  LocalAlgorithms local_algorithms;   //声明局部规划算法对象  这里该类是一个ros类用于调用proto文件中的内容
  std::string full_path = ros::package::getPath("roborts_planning") + "/local_planner/config/local_planner.prototxt"; //声明字符串变量来保存完整proto文件路径
  roborts_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms);    //以只读的方式打开文件并且将内容保存到local_algorithm指向的内存中
  if (&local_algorithms == nullptr) {   //如果在读取完文件之后 local_algorithms指向的还是空则报错 无法读取到文件
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                                   "Cannot load local planner protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();    //该变量是用于标记选择了哪一种局部规划算法的  对应于proto文件中的数字
  frequency_ = local_algorithms.frequency();    //控制频率？。。。。
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));   //坐标转换的时间间隔 或者是坐标转换的方法

  std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_local_plan.prototxt";   //同样的方式记录代价地图的路径
  local_cost_ = std::make_shared<roborts_costmap::CostmapInterface>("local_costmap",
                                                                          *tf_,
                                                                          map_path.c_str());    //将读取出来的局部代价地图存在local_cost指向的内存中
  local_planner_ = roborts_common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(selected_algorithm_);    //这个算法工厂就又是百年大计。。。 与gp调用A*算法的时候是同一个（其实只是用来查找算法的。。） 输入算法对应的名称会返回指向该算法的指针
  if (local_planner_== nullptr) {   //local_planner：轨迹发布器 应该是将规划好的局部路径发布出去的
    ROS_ERROR("global planner algorithm instance can't be loaded");   //如果算法工程返回的指针为空（说明没有找到对应的算法 则报错
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                     "local planner algorithm instance can't be loaded");
  }

  std::string name; //?什么鬼 这个好像没啥用啊
  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = LocalVisualizationPtr(new LocalVisualization(local_planner_nh_, visual_frame_));
  vel_pub_ = local_planner_nh_.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 5);
  //上面三行用于处理路径可视化的初始化。。这一套是ros相关的目前还没搞定
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

void LocalPlannerNode::ExcuteCB(const roborts_msgs::LocalPlannerGoal::ConstPtr &command) {  //as_.start()跳转过来 开始执行服务端的操作
  roborts_common::ErrorInfo error_info = GetErrorInfo();    //先更新一下当前节点的错误信息
  NodeState node_state = GetNodeState();                    //一样 更新一下当前节点的状态 如果是第一轮运行应该是刚被初始化为空闲状态

  if (node_state == NodeState::FAILURE) {                   //如果当前节点的状态是已经失败
    roborts_msgs::LocalPlannerFeedback feedback;            //声明消息变量 作为反馈 （反馈错误信息）
    roborts_msgs::LocalPlannerResult result;                //同样消息变量  用来发布结果
    feedback.error_code = error_info.error_code();
    feedback.error_msg  = error_info.error_msg();
    result.error_code   = feedback.error_code;
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_ERROR("Initialization Failed, Failed to execute action!");  //简单来说就是如果出现了错误就又这边声明的两个变量来返回问题和结果
    return;     //退出
  }
  if (plan_mtx_.try_lock()) {   //这边应该是 如果没锁上就上锁 如果锁上了就不在上锁
    local_planner_->SetPlan(command->route, local_goal_);   //更新或者初始化计划？    ps：这边的command指针是传入参数。。
    plan_mtx_.unlock();   //解锁。。？
    plan_condition_.notify_one();   //？
  }

  ROS_INFO("Send Plan!");           //终端消息 这边就直接发送了？ 感觉有点不对啊
  if (node_state == NodeState::IDLE) {    //如果当前节点是空闲的 就开始规划路径
    StartPlanning();
  }

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (as_.isPreemptRequested()) {
      ROS_INFO("Action Preempted");
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        break;
      } else {
        as_.setPreempted();
        StopPlanning();
        break;
      }
    }
    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING|| node_state == NodeState::SUCCESS
        || node_state == NodeState::FAILURE) {
      roborts_msgs::LocalPlannerFeedback feedback;
      roborts_msgs::LocalPlannerResult result;
      if (!error_info.IsOK()) {
        feedback.error_code = error_info.error_code();
        feedback.error_msg = error_info.error_msg();
        SetErrorInfo(roborts_common::ErrorInfo::OK());

        as_.publishFeedback(feedback);
      }
      if(node_state == NodeState::SUCCESS) {
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        StopPlanning();
        break;
      } else if(node_state == NodeState::FAILURE) {
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        StopPlanning();
        break;
      }
    }
  }

}

void LocalPlannerNode::Loop() {   //该函数处于lp规划的单独规划线程中    //这一块还是比较蒙蔽  2.13继续

  roborts_common::ErrorInfo error_info = local_planner_->Initialize(local_cost_, tf_, visual_);   //完成在进行局部路径规划之前必要的初始化 加载参数配置文件还有一些其他参数的计算
  if (error_info.IsOK()) {
    ROS_INFO("local planner algorithm initialize completed.");    //若初始化成功则 终端消息
  } else {
    ROS_WARN("local planner algorithm initialize failed.");   //如果失败
    SetNodeState(NodeState::FAILURE);   //设置节点状态为失败
    SetErrorInfo(error_info);   //更新错误信息
  }
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);    //线程休眠
  int error_count = 0;  //错误次数

  while (GetNodeState() == NodeState::RUNNING) {    //如果节点正常运行
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();
    roborts_common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
    int need_time = 1000 /frequency_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;

    if (sleep_time <= std::chrono::milliseconds(0)) {
      //LOG_WARNING << "The time planning once is " << cost_time.count() << " beyond the expected time "
        //        << std::chrono::milliseconds(50).count();
      sleep_time = std::chrono::milliseconds(0);
      //SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }

    if (error_info.IsOK()) {
      error_count = 0;
      vel_pub_.publish(cmd_vel_);
      if (local_planner_->IsGoalReached()) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (error_count > max_error_ && max_error_ >0) {
      ROS_WARN("Can not finish plan with max retries( %d  )", max_error_ );
      error_info =  roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::FAILURE);
    } else {
      error_count++;
      ROS_ERROR("Can not get cmd_vel for once. %s error count:  %d", error_info.error_msg().c_str(), error_count);
    }

    SetErrorInfo(error_info);
  }

  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.angular.z = 0;

  cmd_vel_.accel.linear.x = 0;
  cmd_vel_.accel.linear.y = 0;
  cmd_vel_.accel.angular.z = 0;
//  for (int i = 0; i < 10; ++i) {
    vel_pub_.publish(cmd_vel_);
//    usleep(5000);
//  }
}

void LocalPlannerNode::SetErrorInfo(const roborts_common::ErrorInfo error_info) {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(const roborts_common::NodeState& node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

roborts_common::NodeState LocalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

roborts_common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);    //一样的操作 上锁（高级的锁操作会随着对象的消亡自动解锁）
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning() {    //开始规划路径
  if (local_planner_thread_.joinable()) {   //如果规划用的线程是可以加入的（感觉加入这个说法有问题。。
    local_planner_thread_.join();           //就进入这个线程
  }
  //这边的话只有当线程退出之后才有可能到达（或者是第一轮循环
  SetNodeState(roborts_common::NodeState::RUNNING);   //将当前节点的状态设置为正在运行
  local_planner_thread_= std::thread(std::bind(&LocalPlannerNode::Loop,this));    //然后开启新的线程  往loop函数跳转
}

void LocalPlannerNode::StopPlanning() {
  SetNodeState(roborts_common::IDLE);
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info) {
  SetErrorInfo(algorithm_error_info);
}

} // namespace roborts_local_planner

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {   //局部规划节点的入口函数

  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);    //两个用于捕捉系统信号的函数    等种完树再查
  ros::init(argc, argv, "local_planner_node", ros::init_options::NoSigintHandler);    //ros初始化 再调用ros函数之前必备

  roborts_local_planner::LocalPlannerNode local_planner;    //对象实例化 估计和gp一样所有的操作都放在构造函数里面

  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  local_planner.StopPlanning();

  return 0;
}
