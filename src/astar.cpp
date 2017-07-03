#include <iostream>
#include <algorithm>
#include <fcntl.h>
#include "astar.h"

std::vector<VisgraphPoint> AStarMethod::CalShortestPath(VisgraphPoint& start, VisgraphPoint& goal)
{
	//用于存储最终路径
	std::vector<VisgraphPoint> path_vec;
	vis_graph_->Build(start, goal);
	auto& graph = vis_graph_->graph_;
	
	//对start node进行操作
	VisgraphNode start_node(start);
	closed_list_.push_back(start);
	for(auto& e : graph[start])
	{
		VisgraphPoint point;
		point = (start == e.p1_)?e.p2_:e.p1_; 
		VisgraphNode node(point, start);
		//计算启发函数
		float node_h = point.distance(goal);
		node.set_G_H_(e.edge_cost_,node_h);
		open_list_.push_back(node);
		std::sort(open_list_.begin(),open_list_.end());
	}
	
	bool terminate = false;
	while((!terminate) && (!open_list_.empty()))
	{
		//判断goal是否在open_list_中
		VisgraphNode goal_node(goal);
		auto iterator = std::find(open_list_.begin(),open_list_.end(),goal_node);
		if(iterator != open_list_.end())
		{
			terminate = true;
			continue;
		}
		VisgraphNode current_node = open_list_[0];
		
		open_list_.erase(open_list_.begin());
		closed_list_.push_back(current_node);
		//将当前点的邻接点添加到open_list_中
		for(auto& e : graph[current_node.point_])
		{
			VisgraphPoint point;
			point = (current_node == e.p1_)?e.p2_:e.p1_;
			VisgraphNode node(point,current_node.point_);

			//判断是否位于closed_list_中，若是则跳过
			auto iterator = std::find(closed_list_.begin(),closed_list_.end(),node);
			if(iterator != closed_list_.end())
			{
				continue;
			}
			//设置node的F G H
			float node_h = point.distance(goal);
			node.set_G_H_(e.edge_cost_+current_node.G_,node_h);
			//查找该点是否位于open_list_中
			iterator = std::find(open_list_.begin(),open_list_.end(),node);
			//若位于open_list_中
			if(iterator != open_list_.end())
			{
				if(iterator->G_ > node.G_)
				{
					iterator->set_G_H_(node.G_,node.H_);
					continue;
				}
				else
				{
					continue;
				}
			}
			open_list_.push_back(node);
		}
		std::sort(open_list_.begin(),open_list_.end());

		
	}
	if(!terminate && open_list_.empty())
	{
		std::cout << "There is no path from start to goal!" << std::endl;
		return std::vector<VisgraphPoint>();
	}
	//将path逆向导出

	VisgraphNode goal_node(goal);
	auto iterator = std::find(open_list_.begin(),open_list_.end(),goal_node);
	if(iterator != open_list_.end())
	{
		path_vec.push_back(goal);
		VisgraphNode current_node(iterator->parent_p_);
		while(current_node != start_node)
		{
			iterator = std::find(closed_list_.begin(),closed_list_.end(),current_node);
			if(iterator != closed_list_.end())
			{
				current_node.point_ = iterator->parent_p_;
				path_vec.push_back(iterator->point_);
				closed_list_.erase(iterator);
			}
			else
			{
				std::cout << "Error to find the path!" << std::endl;
				return std::vector<VisgraphPoint>();
			}
		}
		path_vec.push_back(start);
		
		std::cout << "The path is: " << std::endl;
		for(auto p = path_vec.rbegin(); p != path_vec.rend(); p++)
		{
			std::cout << p->x_ << ',' << p->y_ << std::endl;
		}
	}
	return path_vec;
// 	std::cout << "End!" << std::endl;
// 	for(auto& node : open_list_)
// 	{
// 		std::cout << "node is:" << node.point_.x_ << ',' << node.point_.y_ << std::endl;
// 		std::cout << "parent is:" << node.parent_p_.x_ << ',' << node.parent_p_.y_ << std::endl;
// 		std::cout << "F is:" << node.F_ << std::endl;
// 	}

}
