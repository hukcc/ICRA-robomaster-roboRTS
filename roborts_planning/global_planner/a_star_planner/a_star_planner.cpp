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
																																//传入起点 终点对应的编号  传入路径容器
			if ( error_info.IsOK() ){
				path.back().pose.orientation = goal.pose.orientation;
				path.back().pose.position.z = goal.pose.position.z;
			}
		}

	}


	return error_info;
}

ErrorInfo AStarPlanner::SearchPath(const int &start_index,                              //所以A*算法的具体内容都在这一个函数里面    前面对于起点终点的确认与A*基本无关
																	 const int &goal_index,
																	 std::vector<geometry_msgs::PoseStamped> &path) {     //对于传入变量作引用  就可以直接改变对应的变量

	g_score_.clear();   //连续清空四个容器  分别是：1.从起点到现在所在格走过的路径对应的分数？
	f_score_.clear();                         //2.是上面g-score与启发式函数的加和
	parent_.clear();                          //3.对应格子的父母格子
	state_.clear();                           //4.标志每一个格子的状态
	gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();		//与调用他的函数一样 获取2D地图的x长度
	gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();		//获取2D地图的y长度
	ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);		//终端消息	标志2D地图的大小
	cost_ = costmap_ptr_->GetCostMap()->GetCharMap();					//获取代价地图对应的行走代价
	g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
	f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
	parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
	state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);				//重置了四个容器的大小	变为与地图格子数一样大	重新赋值为最大值

	std::priority_queue<int, std::vector<int>, Compare> openlist;		//声明了一个优先队列 数据类型int 用vector<int>来储存 比较方法是compare（看谁的f-score比较大就排在前面） ps：还是个优先队列	优先级高的会优先出队（并不是按照先进先出的原则 更像是一个排序堆）
	g_score_.at(start_index) = 0;										//访问g-score对应于起点的位置 并且把他置为0
	openlist.push(start_index);											//将对应与起点的 格子编号存入优先队列

	std::vector<int> neighbors_index;									//声明一个容器用来存储 相邻的格子？的标号
	int current_index, move_cost, h_score, count = 0;					//当前位置编号 移动消耗 h-score（不清楚） 计数器

	while (!openlist.empty()) {											//当优先队列不为空的时候 	当不为空时
		current_index = openlist.top();		//当前位置为堆顶（队尾）	也就是这个优先队列中f-score最大的点 获取这里的编号
		openlist.pop();						//弹出堆顶的值 
		state_.at(current_index) = SearchState::CLOSED;			//将这一点标志为已经使用

		if (current_index == goal_index) {						//如果搜索到了终点 则输出搜索的轮数
			ROS_INFO("Search takes %d cycle counts", count);
			break;
		}

		GetNineNeighbors(current_index, neighbors_index);		//把当前点的相邻点标号放入

		for (auto neighbor_index : neighbors_index) {			//遍历所有的相邻点

			if (neighbor_index < 0 ||
					neighbor_index >= gridmap_height_ * gridmap_width_) {		//跳过超出地图的点
				continue;
			}

			if (cost_[neighbor_index] >= inaccessible_cost_ ||
					state_.at(neighbor_index) == SearchState::CLOSED) {			//跳过不可到达的点
				continue;
			}

			GetMoveCost(current_index, neighbor_index, move_cost);				//计算从当前点到达相邻点的代价	ps：一个中心位置的格子有8个相邻的格子斜的也算

			if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {		//一开始是将score全部最大化了	这里判断条件相当于是在筛选最小的代价方案

				g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];	//筛选完顺便赋值最小的给g-score
				parent_.at(neighbor_index) = current_index;		//将当前结点标记为移动到的下一个点的父结点

				if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {	//如果这个新的格子（相邻格）是没有走过的	
					GetManhattanDistance(neighbor_index, goal_index, h_score);	//启发函数！！！！！曼哈顿式	计算从移动到的下一个点到终点之间的启发函数值
					f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;	//将上面的代价和启发值都计入新的f-score中
					openlist.push(neighbor_index);											//并将新的位置放入优先队列中
					state_.at(neighbor_index) = SearchState::OPEN;							//将这个点的状态标记为可到达（或者是已经规划过	现在来看是不会经过相同格子的不存在路线交叉
				}
			}
		}
		count++;		//计数器	轮数
	}

	if (current_index != goal_index) {											//如果总是无法规划出到达终点的路径	则报错
		ROS_WARN("Global planner can't search the valid path!");
		return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
	}

	unsigned int iter_index = current_index, iter_x, iter_y;		//？？？？？？

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
			abs(neighbor_index - current_index) == gridmap_width_) {		//四个直的方向 代价为10
		move_cost = 10;
	} else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
			abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {		//四个斜着的方向 代价为14
		move_cost = 14;
	} else {
		return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
										 "Move cost can't be calculated cause current neighbor index is not accessible");		//如果是错误的点 则报错
	}
	return ErrorInfo(ErrorCode::OK);
}

void AStarPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
	manhattan_distance = heuristic_factor_* 10 * (abs(index1 / gridmap_width_ - index2 / gridmap_width_) +
			abs(index1 % gridmap_width_ - index2 % gridmap_width_));
}

void AStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
	neighbors_index.clear();		//总之先清空相邻的格子的容器（上一轮留下来的）	然后分别讨论当前点在2D地图的什么位置 接着将他相邻的点放进相邻点容器中
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
