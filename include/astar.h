#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <map>
#include <unordered_set>
#include <set>
#include <vector>
#include <functional>
#include "visgraph.h"

class VisgraphNode
{
public:
	VisgraphPoint point_;
	VisgraphPoint parent_p_;
	float F_;
	float G_;
	float H_;
	VisgraphNode(const VisgraphPoint& point):point_(point)
	{
		F_ = 0;
		G_ = 0;
		H_ = 0;
	}
	VisgraphNode(const VisgraphPoint& point, const VisgraphPoint& parent_p):point_(point),parent_p_(parent_p)
	{
		F_ = 0;
		G_ = 0;
		H_ = 0;
	}
	virtual ~VisgraphNode()
	{}
	void set_parent_p_(const VisgraphPoint& parent_p)
	{
		parent_p_ = parent_p;
	}
	void set_G_H_(const float _G, const float _H)
	{
		G_ = _G;
		//
		H_ = 0;
		F_ = G_ + H_;
	}
};

inline bool operator<(const VisgraphNode& lnode, const VisgraphNode& rnode)
{
	return lnode.F_ < rnode.F_;
}

inline bool operator==(const VisgraphNode& lnode, const VisgraphNode& rnode)
{
	return lnode.point_ == rnode.point_;
}

inline bool operator!=(const VisgraphNode& lnode, const VisgraphNode& rnode)
{
	return !operator==(lnode, rnode);
}

class HashNode
{
public:
	std::size_t operator()(const VisgraphNode &key) const
	{
		using std::hash;
		return hash<float>()(key.F_);
	}
};

class AStarMethod
{
public:
	AStarMethod(VisgraphGraph* graph):vis_graph_(graph)
	{}
	virtual ~AStarMethod()
	{
// 		delete vis_graph_;
	}
	std::vector<VisgraphPoint> CalShortestPath(VisgraphPoint& start, VisgraphPoint& goal);
	
private:
	std::vector<VisgraphNode> open_list_;
	std::vector<VisgraphNode> closed_list_;
	VisgraphGraph* vis_graph_;
};

#endif