#ifndef __VISGRAPH_H__
#define __VISGRAPH_H__

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <cmath>
#include <boost/concept_check.hpp>

class VisgraphPoint
{
public:
	float x_;
	float y_;
	int polygon_id_;
	VisgraphPoint(float _x=0, float _y=0, int _polygon_id=0):x_(_x),y_(_y),polygon_id_(_polygon_id)
	{}
	virtual ~VisgraphPoint()
	{}
	
	float distance(const VisgraphPoint& point)
	{
		return sqrt(pow(this->x_ - point.x_,2) + pow(this->y_ - point.y_,2));
	}
	
// 	VisgraphPoint& operator=(const VisgraphPoint& rpoint)
// 	{
// 		this->x_ = rpoint.x_;
// 		this->y_ = rpoint.y_;
// 		this->polygon_id_ = rpoint.polygon_id_;
// 		return *this;
// 	}
};

class VisgraphEdge
{
public:
	VisgraphPoint p1_;
	VisgraphPoint p2_;
	float edge_cost_;
	VisgraphEdge(VisgraphPoint& _p1, VisgraphPoint& _p2):p1_(_p1),p2_(_p2)
	{
		edge_cost_ = p1_.distance(p2_);
	}
	virtual ~VisgraphEdge()
	{} 
// 	VisgraphEdge& operator=(const VisgraphEdge& redge)
// 	{
// 		this->p1_ = redge.p1_;
// 		this->p2_ = redge.p2_;
// 		this->edge_cost_ = redge.edge_cost_;
// 		return *this;
// 	}
};

class HashPoint  
{
public:
    std::size_t operator()(const VisgraphPoint &key) const   
    {  
		using std::hash;
        return ((hash<float>()(key.x_))  
              ^ (hash<float>()(key.y_)));  
    }  
}; 

class HashEdge
{
public:
	std::size_t operator()(const VisgraphEdge &key) const
	{
		using std::hash;
		return ((HashPoint()(key.p1_)
			  ^ (HashPoint()(key.p2_))));
	}
};

inline bool operator==(const VisgraphPoint& lpoint, const VisgraphPoint& rpoint)
{
	return ((lpoint.x_ == rpoint.x_) && (lpoint.y_ == rpoint.y_));
}

inline bool operator!=(const VisgraphPoint& lpoint, const VisgraphPoint& rpoint)
{
	return !operator==(lpoint,rpoint);
}

inline bool operator==(const VisgraphEdge& ledge, const VisgraphEdge& redge)
{
	return (((ledge.p1_ == redge.p1_) && (ledge.p2_ == redge.p2_)) 
	      || (ledge.p1_ == redge.p2_) && (ledge.p2_ == redge.p1_));
}

inline bool operator!=(const VisgraphEdge& ledge, const VisgraphEdge& redge)
{
	return !operator==(ledge,redge);
}


class VisgraphGraph
{
public:
	std::unordered_map<VisgraphPoint, std::unordered_set<VisgraphEdge, HashEdge>, HashPoint> graph_;
	std::unordered_set<VisgraphEdge, HashEdge> edges_;
	std::unordered_map<int, std::unordered_set<VisgraphEdge, HashEdge>> polygons_;
	
	VisgraphGraph(const std::vector<std::vector<VisgraphPoint>>& polygons);
	virtual ~VisgraphGraph()
	{}
	void Build(VisgraphPoint& start, VisgraphPoint& goal);
	bool Intersect(const VisgraphPoint& _p1, const VisgraphPoint& _p2);
};
#endif