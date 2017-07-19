#include <iostream>

#include "visgraph.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <map>
#include <cmath>

VisgraphGraph::VisgraphGraph(const std::vector<std::vector<VisgraphPoint>>& polygons)
{
	//创建factory
	PrecisionModel *pm = new PrecisionModel();
	geometry_factory_ = GeometryFactory::create(pm, -1);
	delete pm;
	
	obstacle_polygons_.clear();
	LinearRing* boundary_line;
	std::vector<Geometry *> *hole_line = new std::vector<Geometry *>;
	for(int i=0; i<polygons.size(); i++)
	{
		CoordinateSequence *cl = new CoordinateArraySequence();
		cl->add(Coordinate(polygons.at(i).at(0).x_, polygons.at(i).at(0).y_));
		for(int j=0; j<polygons.at(i).size(); j++)
		{
			if(j < polygons.at(i).size()-1)
			{
				VisgraphPoint p1(polygons.at(i).at(j));
				VisgraphPoint p2(polygons.at(i).at(j+1));
				VisgraphEdge edge(p1,p2);
				cl->add(Coordinate(polygons.at(i).at(j+1).x_, polygons.at(i).at(j+1).y_));
				polygons_[i].insert(edge);
			}
			else
			{
				VisgraphPoint p1(polygons.at(i).at(j));
				VisgraphPoint p2(polygons.at(i).at(0));
				VisgraphEdge edge(p1,p2);
				cl->add(Coordinate(polygons.at(i).at(0).x_, polygons.at(i).at(0).y_));
				polygons_[i].insert(edge);
			}
		}
		LinearRing *lr = geometry_factory_->createLinearRing(cl);
		std::vector<Geometry *> *holes = new std::vector<Geometry *>;
		if(polygons.at(i).at(0).polygon_id_ != -2)
		{
			Polygon *poly = geometry_factory_->createPolygon(lr, holes);
			hole_line->push_back(lr);
			obstacle_polygons_.push_back(poly);
		}
		else
		{
			boundary_polygon_ = geometry_factory_->createPolygon(lr, holes);
			boundary_line     = lr;
		}
	}
	obstacle_collection_ = geometry_factory_->createGeometryCollection(obstacle_polygons_);
	whole_polygon_ = geometry_factory_->createPolygon(boundary_line,hole_line);
	if(whole_polygon_->isValid())
	{
		whole_polygon_isvalid_ = true;
	}
	else
	{
		whole_polygon_isvalid_ = false;
	}
}

void VisgraphGraph::Build(VisgraphPoint& start, VisgraphPoint& goal)
{
	bool start_in = false;
	bool goal_in  = false;
	start.polygon_id_ = -1;
	goal.polygon_id_  = -1;
	
	//为了能够重复建图，每次对于不同的起始终止点都得清楚edges和graph
	graph_.clear();
	edges_.clear();
	
	//判断起始点或目标点是否在障碍物的边或顶点上，是的话，进行排除
#ifndef GEOS_DEBUG
	for(auto& poly : polygons_)
	{
		for(auto& e : poly.second)
		{
			float __t = (start.x_ - e.p1_.x_)/(e.p2_.x_ - e.p1_.x_);
			float __y = __t*(e.p2_.y_ - e.p1_.y_) + e.p1_.y_;
			if(__t >= 0 && __t <= 1 && __y == start.y_)
			{
				return;
			}
			
			__t = (goal.x_ - e.p1_.x_)/(e.p2_.x_ - e.p1_.x_);
			__y = __t*(e.p2_.y_ - e.p1_.y_) + e.p1_.y_;
			if(__t >= 0 && __t <= 1 && __y == goal.y_)
			{
				return;
			}
		}
	}
#endif

	for(const auto& edges : polygons_)
	{
		for(const auto& edge : edges.second)
		{
			VisgraphPoint p1(edge.p1_);
			VisgraphPoint p2(edge.p2_);
			graph_[p1].insert(edge);
			graph_[p2].insert(edge);
			edges_.insert(edge);
		}
	}
	//先将start和goal添加到graph中
	for(const auto& point : graph_)
	{
		if(!Intersect(start,point.first) && !start_in)
		{
			VisgraphPoint point1(point.first);
			VisgraphEdge edge(start,point1);
			graph_[start].insert(edge);
			graph_[point.first].insert(edge);
			edges_.insert(edge);
			start_in = true;
		}
		if(!Intersect(goal,point.first) && !goal_in)
		{
			VisgraphPoint point1(point.first);
			VisgraphEdge edge(point1,goal);
			graph_[goal].insert(edge);
			graph_[point.first].insert(edge);
			edges_.insert(edge);
			goal_in = true;
		}
		if(start_in && goal_in)
			break;
	}
	int compute_count = 0;
	//遍历所有点，进行判断并且连接有效边
	for(const auto& point1 : graph_)
	{
		for(const auto& point2 : graph_)
		{
			if(point1.first != point2.first)
			{
				VisgraphEdge e1(point1.first,point2.first);
				VisgraphEdge e2(point2.first,point1.first);
				auto index1 = edges_.find(e1);
				auto index2 = edges_.find(e2);
				if(index1 == edges_.end() && index2 == edges_.end())
				{
					if(!Intersect(point1.first,point2.first))
					{
						VisgraphPoint point11(point1.first);
						VisgraphPoint point22(point2.first);
						VisgraphEdge edge(point11,point22);
						graph_[point1.first].insert(edge);
						graph_[point2.first].insert(edge);
						edges_.insert(edge);
						compute_count ++;
					}
				}
			}

		}
	}
	std::cout << "The compute count is: " << compute_count << std::endl;
}

bool VisgraphGraph::Intersect(const VisgraphPoint& _p1, const VisgraphPoint& _p2)
{
	CoordinateSequence *cl = new CoordinateArraySequence();
	
	Coordinate coord1 = Coordinate(_p1.x_,_p1.y_);
	Coordinate coord2 = Coordinate(_p2.x_,_p2.y_);
	
	Point *point1 = geometry_factory_->createPoint(coord1);
	Point *point2 = geometry_factory_->createPoint(coord2);
	cl->add(Coordinate(coord1));
	cl->add(Coordinate(coord2));
	LineString* line = geometry_factory_->createLineString(cl); 
	
	if(boundary_polygon_)
	{
		if(whole_polygon_isvalid_)
		{
			if(!whole_polygon_->contains(line))
			{
				return true;
			}
		}
		else
		{
			if(!boundary_polygon_->contains(line))
			{
				return true;
			}
			if(obstacle_collection_->intersects(line))
			{
				if(obstacle_collection_->touches(line))
				{
					return false;
				}
				return true;
			}
		}

	}
	return false;
}

#ifndef GEOS_DEBUG
bool VisgraphGraph::Intersect(const VisgraphPoint& _p1, const VisgraphPoint& _p2)
{
	//先对多边形障碍物的内点连接进行排查
	if(_p1.polygon_id_ == _p2.polygon_id_ && _p1.polygon_id_ != -1 && _p1.polygon_id_ != -2)
	{
		float x_mean = 0.5*_p1.x_ + 0.5*_p2.x_;
		float y_mean = 0.5*_p1.y_ + 0.5*_p2.y_;
		
		int left_count  = 0;
		int right_count = 0;
		
		//多边形内点判断法
		for(const auto& e : polygons_[_p1.polygon_id_])
		{
			float disy1 = e.p1_.y_ - y_mean;
			float disy2 = e.p2_.y_ - y_mean;
			if((_p1 == e.p1_&&_p2 == e.p2_)||((_p1 == e.p2_)&&(_p2 == e.p1_)))
			{
				return true;
			}
			//用射线法来判断
			if((disy1*disy2) < 0)
			{
				float t = (y_mean - e.p2_.y_)/(e.p1_.y_ - e.p2_.y_);
				float insersect_x = t*e.p1_.x_ + (1-t)*e.p2_.x_;
				if(insersect_x == x_mean)
				{
					return true;
				}
				if(insersect_x > x_mean)
				{
					right_count++;
				}
				else if(insersect_x < x_mean)
				{
					left_count++;
				}
			}
			//暂时不考虑奇葩情况
			else if((disy1*disy2) == 0)
			{
				return true;
			}
		}
		if((right_count%2)&&(left_count%2))
		{
			return true;
		}
	}
	//对边界的内点进行排除
	else if(_p1.polygon_id_ == _p2.polygon_id_ && _p1.polygon_id_ == -2)
	{
		float x_mean = 0.5*_p1.x_ + 0.5*_p2.x_;
		float y_mean = 0.5*_p1.y_ + 0.5*_p2.y_;
		VisgraphPoint mean_point(x_mean, y_mean);
		
		for(int i = 0; i < polygons_.size(); i++)
		{
			auto e = polygons_[i].begin();
			if(e->p1_.polygon_id_ == -2)
			{
				if(IsOutside(mean_point,polygons_[i]))
				{
					return true;
				}
				break;
			}
		}
	}
	for(int i = 0; i < polygons_.size(); i++)
	{
		for(const auto& e : polygons_[i])
		{
			if(e.p1_.polygon_id_ != -2)
			{
				VisgraphPoint _p3(e.p1_);
				VisgraphPoint _p4(e.p2_);
				//判断一个点是否在另一个多边形内部
				if(_p1.polygon_id_ != _p2.polygon_id_)
				{
					int left_count  = 0;
					int right_count = 0;
					if(_p1.polygon_id_ >= 0)
					{
						for(const auto& e : polygons_[_p1.polygon_id_])
						{
							float x_mean = _p2.x_;
							float y_mean = _p2.y_;
							float disy1 = e.p1_.y_ - y_mean;
							float disy2 = e.p2_.y_ - y_mean;
							//用射线法来判断
							if((disy1*disy2) < 0)
							{
								float t = (y_mean - e.p2_.y_)/(e.p1_.y_ - e.p2_.y_);
								float insersect_x = t*e.p1_.x_ + (1-t)*e.p2_.x_;
								if(insersect_x == x_mean)
								{
									return true;
								}
								if(insersect_x > x_mean)
								{
									right_count++;
								}
								else if(insersect_x < x_mean)
								{
									left_count++;
								}
							}
							//暂时不考虑奇葩情况
		// 					else if((disy1*disy2) == 0)
		// 					{
		// 						return true;
		// 					}
						}
						if((right_count%2)&&(left_count%2))
						{
							return true;
						}
					}
					left_count  = 0;
					right_count = 0;
					if(_p2.polygon_id_ >= 0)
					{
						for(const auto& e : polygons_[_p2.polygon_id_])
						{
							float x_mean = _p1.x_;
							float y_mean = _p1.y_;
							float disy1 = e.p1_.y_ - y_mean;
							float disy2 = e.p2_.y_ - y_mean;
							//用射线法来判断
							if((disy1*disy2) < 0)
							{
								float t = (y_mean - e.p2_.y_)/(e.p1_.y_ - e.p2_.y_);
								float insersect_x = t*e.p1_.x_ + (1-t)*e.p2_.x_;
								if(insersect_x == x_mean)
								{
									return true;
								}
								if(insersect_x > x_mean)
								{
									right_count++;
								}
								else if(insersect_x < x_mean)
								{
									left_count++;
								}
							}
							//暂时不考虑奇葩情况
		// 					else if((disy1*disy2) == 0)
		// 					{
		// 						return true;
		// 					}
						}
						if((right_count%2)&&(left_count%2))
						{
							return true;
						}
					}
				}
				//有交点的情况
				if((_p1.x_-_p2.x_)/(_p1.y_-_p2.y_) != (_p3.x_-_p4.x_)/(_p3.y_-_p4.y_))
				{
					Eigen::Matrix2f matrix_l;
					matrix_l << _p1.x_-_p2.x_, _p4.x_-_p3.x_,
								_p1.y_-_p2.y_, _p4.y_-_p3.y_;
					Eigen::Vector2f vector_r(_p4.x_-_p2.x_, _p4.y_-_p2.y_);
					Eigen::Vector2f vector_l = matrix_l.inverse()*vector_r;
					if(vector_l[0]>0 && vector_l[0]<1 && vector_l[1]>0 && vector_l[1]<1)
					{
						return true;
					}
				}
				else
				{
					//同样，对奇葩情况进行排除
					float t = (_p3.y_ - _p2.y_)/(_p1.y_ - _p2.y_);
					float insersect_x = t*_p1.x_ + (1-t)*_p2.x_;
					if(insersect_x == _p3.x_)
					{
						return true;
					}
				}
			}
		}
	}
	for(int i = 0; i < polygons_.size(); i++)
	{
		auto e = polygons_[i].begin();
		if(e->p1_.polygon_id_ == -2)
		{
			VisgraphEdge edge(_p1,_p2);
			if(IsIntersect(edge,polygons_[i]))
			{
				return true;
			}
			break;
		}
	}
	return false;
}

//原则上用于判断危险，遵循宁杀错不放过的原理
bool VisgraphGraph::IsInside(const VisgraphPoint& _p, const std::unordered_set<VisgraphEdge, HashEdge>& _polygon)
{
	for(auto& e : _polygon)
	{
		float __t = (_p.x_ - e.p1_.x_)/(e.p2_.x_ - e.p1_.x_);
		float __y = __t*(e.p2_.y_ - e.p1_.y_) + e.p1_.y_;
		if(__t >= 0 && __t <= 1 && __y == _p.y_)
		{
			return true;
		}
	}
	int left_count  = 0;
	int right_count = 0;
	for(const auto& e : _polygon)
	{
		float x_mean = _p.x_;
		float y_mean = _p.y_;
		float disy1 = e.p1_.y_ - y_mean;
		float disy2 = e.p2_.y_ - y_mean;
		//用射线法来判断
		if((disy1*disy2) < 0)
		{
			float t = (y_mean - e.p2_.y_)/(e.p1_.y_ - e.p2_.y_);
			float insersect_x = t*e.p1_.x_ + (1-t)*e.p2_.x_;
			if(insersect_x == x_mean)
			{
				return true;
			}
			if(insersect_x > x_mean)
			{
				right_count++;
			}
			else if(insersect_x < x_mean)
			{
				left_count++;
			}
		}
		else if((disy1*disy2) == 0)
		{
			return true;
		}
	}
	if((right_count%2) && (left_count%2))
	{
		return true;
	}
	return false;
}

//用于危险判断
bool VisgraphGraph::IsOutside(const VisgraphPoint& _p, const std::unordered_set< VisgraphEdge, HashEdge >& _polygon)
{
    for(auto& e : _polygon)
	{
		float __t = (_p.x_ - e.p1_.x_)/(e.p2_.x_ - e.p1_.x_);
		float __y = __t*(e.p2_.y_ - e.p1_.y_) + e.p1_.y_;
		if(__t >= 0 && __t <= 1 && __y == _p.y_)
		{
			return true;
		}
	}
	int left_count  = 0;
	int right_count = 0;
	for(const auto& e : _polygon)
	{
		float x_mean = _p.x_;
		float y_mean = _p.y_;
		float disy1 = e.p1_.y_ - y_mean;
		float disy2 = e.p2_.y_ - y_mean;
		//用射线法来判断
		if((disy1*disy2) < 0)
		{
			float t = (y_mean - e.p2_.y_)/(e.p1_.y_ - e.p2_.y_);
			float insersect_x = t*e.p1_.x_ + (1-t)*e.p2_.x_;
			if(insersect_x == x_mean)
			{
				return true;
			}
			if(insersect_x > x_mean)
			{
				right_count++;
			}
			else if(insersect_x < x_mean)
			{
				left_count++;
			}
		}
		else if((disy1*disy2) == 0)
		{
			return true;
		}
	}
	if(!(right_count%2) && !(left_count%2))
	{
		return true;
	}
	return false;
}

//线是否跟边界多边形有交点，即使是与边重合也算有交点，线经过顶点也算有交点
bool VisgraphGraph::IsIntersect(const VisgraphEdge& _e, const std::unordered_set< VisgraphEdge, HashEdge >& _polygon)
{
	VisgraphPoint _p1 = _e.p1_;
	VisgraphPoint _p2 = _e.p2_;
	for(auto& e : _polygon)
	{
		VisgraphPoint _p3 = e.p1_;
		VisgraphPoint _p4 = e.p2_;
		//有交点的情况
		if((_p1.x_-_p2.x_)/(_p1.y_-_p2.y_) != (_p3.x_-_p4.x_)/(_p3.y_-_p4.y_))
		{
			Eigen::Matrix2f matrix_l;
			matrix_l << _p1.x_-_p2.x_, _p4.x_-_p3.x_,
						_p1.y_-_p2.y_, _p4.y_-_p3.y_;
			Eigen::Vector2f vector_r(_p4.x_-_p2.x_, _p4.y_-_p2.y_);
			Eigen::Vector2f vector_l = matrix_l.inverse()*vector_r;
			if(vector_l[0]>=0 && vector_l[0]<=1 && vector_l[1]>=0 && vector_l[1]<=1)
			{
				//如果直线其中某个点与多边形的顶点重合
				if(_p1.x_ == _p3.x_ || _p1.x_ == _p4.x_ || _p2.x_ == _p3.x_ || _p2.x_ == _p4.x_)
				{
					//需要判断直线中心是否在外部
					VisgraphPoint mean_point(0.5*_p1.x_+0.5*_p2.x_, 0.5*_p1.y_+0.5*_p2.y_);
					if(IsOutside(mean_point,_polygon))
					{
						return true;
					}
					continue;
				}
				//没有重合点的其他所有情况均属于直线与多边形有交点的情况
				return true;
			}
		}
		else
		{
			//平行且多边形的边落于直线上，则算作有交点
			float t = (_p3.y_ - _p2.y_)/(_p1.y_ - _p2.y_);
			float insersect_x = t*_p1.x_ + (1-t)*_p2.x_;
			if(insersect_x == _p3.x_)
			{
				return true;
			}
		}
	}
}
#endif
