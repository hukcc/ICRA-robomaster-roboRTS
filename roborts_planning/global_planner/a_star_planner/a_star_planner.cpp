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

#include "a_star_planner.h"

namespace roborts_global_planner{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

AStarPlanner::AStarPlanner(CostmapPtr costmap_ptr) :      //构造函数应该是在 算法工厂那里被搜索到之后就初始化了
    GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),    //这里传入的应该是代价地图参数的文件对应指针 这边的两个指针基本上就是同一个量 不过指针的类型可能不同  这里为了确保一致又重新赋值了一次
    gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),   //getcostmap返回一个指向2D的地图的指针  然后获得x方向上的地图的长度
    gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),    //同理 获得y方向上的地图的长度
    cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {               //获取在地图上行走的代价？

  AStarPlannerConfig a_star_planner_config;   //声明用于接受参数的对象  也不知道这个类是在哪里定义的 大概是ROS的一种操作
  std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/a_star_planner/"\
      "config/a_star_planner_config.prototxt";                                                            //保存文件路径

  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &a_star_planner_config)) {           //从目标文件中读取数据 如果失败就报错
    ROS_ERROR("Cannot load a star planner protobuf configuration file.");
  }
  //  AStarPlanner param config   算法参数初始化
  heuristic_factor_ = a_star_planner_config.heuristic_factor();   //启发因素   应该是加载启发函数的类型
  inaccessible_cost_ = a_star_planner_config.inaccessible_cost(); //难以达到的代价   应该是放弃的阈值
  goal_search_tolerance_ = a_star_planner_config.goal_search_tolerance()/costmap_ptr->GetCostMap()->GetResolution();    //目标搜索误差   ps：resolution在这里是分辨率的意思 可以理解为缩放比例
}

AStarPlanner::~AStarPlanner(){
  cost_ = nullptr;
}

ErrorInfo AStarPlanner::Plan(const geometry_msgs::PoseStamped &start,   //传入参数 起点 终点  用于储存路径的动态数组
                             const geometry_msgs::PoseStamped &goal,
                             std::vector<geometry_msgs::PoseStamped> &path) {     //用A*在2D平面上规划路径

  unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;      //起点 终点 中间点 2D坐标？
  unsigned int valid_goal[2];                                               //有效目标？  相当于一个Point量
  unsigned  int shortest_dist = std::numeric_limits<unsigned int>::max();   //返回编译器允许的unsigned int 的最大值作为最短距离
  bool goal_valid = false;    //目标是否有效  默认无效。。。

  if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,                 //把起点的坐标转化到平面上
                                             start.pose.position.y,
                                             start_x,
                                             start_y)) {                            //如果这里的世界坐标有任何一个小于map上的原点的话就退出 报错
    ROS_WARN("Failed to transform start pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Start pose can't be transformed to costmap frame.");
  }
  if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,                  //把目标点的坐标转化到平面
                                             goal.pose.position.y,
                                             goal_x,
                                             goal_y)) {
    ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Goal pose can't be transformed to costmap frame.");
  }                                                                                   //上面两段确保对应点在地图内
  if (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)<inaccessible_cost_){         //如果到达目标点的代价在可以接受的范围内
    valid_goal[0] = goal_x;
    valid_goal[1] = goal_y;
    goal_valid = true;                                                                //则将其设置为一个合理的目标
  }else{                                                                              //否则
    tmp_goal_x = goal_x;
    tmp_goal_y = goal_y - goal_search_tolerance_;                                     //先将目标的y坐标下移一个搜索误差

    while(tmp_goal_y <= goal_y + goal_search_tolerance_){                             //在y坐标达到上限误差之前
      tmp_goal_x = goal_x - goal_search_tolerance_;                                     
      while(tmp_goal_x <= goal_x + goal_search_tolerance_){                           //双重遍历 在以goal_x 和 goal_y 为中心的正方形内 寻找合适的点
        unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);   //temp点的代价
        unsigned int dist = abs(goal_x - tmp_goal_x) + abs(goal_y - tmp_goal_y);            //与中心点的直线距离
        if (cost < inaccessible_cost_ && dist < shortest_dist ) {       //如果找到了符合标准的点              话说设个算法的启发函数在哪里阿。。。
          shortest_dist = dist;
          valid_goal[0] = tmp_goal_x;     //就改变所有参数  
          valid_goal[1] = tmp_goal_y;
          goal_valid = true;
        }
        tmp_goal_x += 1;
      }//x
      tmp_goal_y += 1;
    }//y
  }
  ErrorInfo error_info;
  if (!goal_valid){
    error_info=ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);     //如果都没有找到 就报错并且清空路径的容器
    path.clear();
  }
  else{   //如果有找到符合条件的点
    unsigned int start_index, goal_index;
    start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
    goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);      //获取按照行优先顺序从原点开始排列中的对应的起点编号和最后规划目标点的编号

    costmap_ptr_->GetCostMap()->SetCost(start_x, start_y,roborts_costmap::FREE_SPACE);    //清空起点处的代价值 （从起点到起点代价为0）

    if(start_index == goal_index){    //当起点与终点重合 停在原地 返回值设置为ok
      error_info=ErrorInfo::OK();
      path.clear();
      path.push_back(start);
      path.push_back(goal);
    }
    else{                             //如果没有到达终点
      error_info = SearchPath(start_index, goal_index, path);   //woc这里才是大头 上面一直都是在控制起点和终点 在找到合适的起点和终点之后 才进入这一条进行实际的路径规划
      if ( error_info.IsOK() ){
        path.back().pose.orientation = goal.pose.orientation;
        path.back().pose.position.z = goal.pose.position.z;
      }
    }

  }


  return error_info;
}

ErrorInfo AStarPlanner::SearchPath(const int &start_index,
                                   const int &goal_index,
                                   std::vector<geometry_msgs::PoseStamped> &path) {

  g_score_.clear();
  f_score_.clear();
  parent_.clear();
  state_.clear();
  gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
  gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
  ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
  cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
  g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

  std::priority_queue<int, std::vector<int>, Compare> openlist;
  g_score_.at(start_index) = 0;
  openlist.push(start_index);

  std::vector<int> neighbors_index;
  int current_index, move_cost, h_score, count = 0;

  while (!openlist.empty()) {
    current_index = openlist.top();
    openlist.pop();
    state_.at(current_index) = SearchState::CLOSED;

    if (current_index == goal_index) {
      ROS_INFO("Search takes %d cycle counts", count);
      break;
    }

    GetNineNeighbors(current_index, neighbors_index);

    for (auto neighbor_index : neighbors_index) {

      if (neighbor_index < 0 ||
          neighbor_index >= gridmap_height_ * gridmap_width_) {
        continue;
      }

      if (cost_[neighbor_index] >= inaccessible_cost_ ||
          state_.at(neighbor_index) == SearchState::CLOSED) {
        continue;
      }

      GetMoveCost(current_index, neighbor_index, move_cost);

      if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {

        g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
        parent_.at(neighbor_index) = current_index;

        if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
          GetManhattanDistance(neighbor_index, goal_index, h_score);
          f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
          openlist.push(neighbor_index);
          state_.at(neighbor_index) = SearchState::OPEN;
        }
      }
    }
    count++;
  }

  if (current_index != goal_index) {
    ROS_WARN("Global planner can't search the valid path!");
    return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
  }

  unsigned int iter_index = current_index, iter_x, iter_y;

  geometry_msgs::PoseStamped iter_pos;
  iter_pos.pose.orientation.w = 1;
  iter_pos.header.frame_id = "map";
  path.clear();
  costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
  costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
  path.push_back(iter_pos);

  while (iter_index != start_index) {
    iter_index = parent_.at(iter_index);
//    if(cost_[iter_index]>= inaccessible_cost_){
//      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned int>(cost_[iter_index]);
//    }
    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);
  }

  std::reverse(path.begin(),path.end());

  return ErrorInfo(ErrorCode::OK);

}

ErrorInfo AStarPlanner::GetMoveCost(const int &current_index,
                                    const int &neighbor_index,
                                    int &move_cost) const {
  if (abs(neighbor_index - current_index) == 1 ||
      abs(neighbor_index - current_index) == gridmap_width_) {
    move_cost = 10;
  } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
      abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
    move_cost = 14;
  } else {
    return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                     "Move cost can't be calculated cause current neighbor index is not accessible");
  }
  return ErrorInfo(ErrorCode::OK);
}

void AStarPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
  manhattan_distance = heuristic_factor_* 10 * (abs(index1 / gridmap_width_ - index2 / gridmap_width_) +
      abs(index1 % gridmap_width_ - index2 % gridmap_width_));
}

void AStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
  neighbors_index.clear();
  if(current_index - gridmap_width_ >= 0){
    neighbors_index.push_back(current_index - gridmap_width_);       //up
  }
  if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
  }
  if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index - 1);        //left
  }
  if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
      && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
  }
  if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
    neighbors_index.push_back(current_index + gridmap_width_);     //down
  }
  if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
      && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
  }
  if(current_index  + 1 < gridmap_width_* gridmap_height_
      && (current_index  + 1 ) % gridmap_width_!= 0) {
    neighbors_index.push_back(current_index + 1);                   //right
  }
  if(current_index - gridmap_width_ + 1 >= 0
      && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
    neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
  }
}

} //namespace roborts_global_planner
