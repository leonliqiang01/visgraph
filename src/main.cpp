#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <chrono>

#include "visgraph.h"
#include "astar.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "visgraph");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	std::vector<std::vector<VisgraphPoint>> polygons;
	
	std::vector<VisgraphPoint> polygon1;
	polygon1.push_back(VisgraphPoint(10,5,0));
	polygon1.push_back(VisgraphPoint(15,6,0));
	polygon1.push_back(VisgraphPoint(13,8,0));
	polygon1.push_back(VisgraphPoint(18,13,0));
	polygon1.push_back(VisgraphPoint(13,16,0));
	polygon1.push_back(VisgraphPoint(8,13,0));
	polygon1.push_back(VisgraphPoint(5,7,0));
	polygons.push_back(polygon1);
	
	std::vector<VisgraphPoint> polygon2;
	polygon2.push_back(VisgraphPoint(22,16,1));
	polygon2.push_back(VisgraphPoint(26,11,1));
	polygon2.push_back(VisgraphPoint(32,12,1));
	polygon2.push_back(VisgraphPoint(30,15,1));
	polygon2.push_back(VisgraphPoint(27,14,1));
	polygon2.push_back(VisgraphPoint(27,19,1));
	polygon2.push_back(VisgraphPoint(23,19.5,1));
	polygons.push_back(polygon2);
	
	std::vector<VisgraphPoint> polygon3;
	polygon3.push_back(VisgraphPoint(19,7,2));
	polygon3.push_back(VisgraphPoint(25,7,2));
	polygon3.push_back(VisgraphPoint(22,12,2));
	polygons.push_back(polygon3);

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	
	VisgraphGraph graph(polygons);
	//要先进行起始点与末端点位置判断
	VisgraphPoint start(0,0);
	VisgraphPoint goal(14,12);
	AStarMethod astart(&graph);

	std::vector<VisgraphPoint> path_vec = astart.CalShortestPath(start,goal);

// 	graph.Build(start, goal);
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
	std::cout << "solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;
	ros::Rate r(30);
	
	float f = 0.0;
	while(ros::ok())
	{
		visualization_msgs::Marker points, line_strip, line_list, line_list2;
		points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world";
		points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
		points.ns = line_strip.ns = line_list.ns = "visgraph";
		points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
		
		line_list2.header.frame_id = "world";
		line_list2.header.stamp = ros::Time::now();
		line_list2.ns = "visgraph";
		line_list2.action = visualization_msgs::Marker::ADD;
		line_list2.pose.orientation.w = 1.0;
		
		points.id = 0;
		line_strip.id = 1;
		line_list.id = 2;
		line_list2.id = 3;
		
		points.type = visualization_msgs::Marker::POINTS;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		line_list2.type = visualization_msgs::Marker::LINE_LIST;
		line_list.type  = visualization_msgs::Marker::LINE_LIST;
		
		points.scale.x = 0.2;
		points.scale.y = 0.2;
		
		line_strip.scale.x = 0.1;
		line_list2.scale.x = 0.1;
		line_list.scale.x  = 0.1;
		
		points.color.g = 1.0f;
		points.color.a = 1.0;
		
		line_strip.color.g = 1.0;
		line_strip.color.a = 1.0;
		
		line_list2.color.b = 1.0;
		line_list2.color.a = 1.0;
		
		line_list.color.r = 1.0;
		line_list.color.a = 1.0;
		
		for(const auto &point : graph.graph_)
		{
			geometry_msgs::Point p;
			p.x = point.first.x_;
			p.y = point.first.y_;
			p.z = 0;
			points.points.push_back(p);
		}
		
		for(const auto &edges : graph.polygons_)
		{
			for(const auto &edge : edges.second)
			{
				geometry_msgs::Point p;
				p.x = edge.p1_.x_;
				p.y = edge.p1_.y_;
				p.z = 0;
				line_list2.points.push_back(p);
				p.x = edge.p2_.x_;
				p.y = edge.p2_.y_;
				p.z = 0;
				line_list2.points.push_back(p);
			}
		}
		
		for(const auto &e : graph.edges_)
		{
			geometry_msgs::Point p;
			p.x = e.p1_.x_;
			p.y = e.p1_.y_;
			p.z = 0;
			line_list.points.push_back(p);
			p.x = e.p2_.x_;
			p.y = e.p2_.y_;
			p.z = 0;
			line_list.points.push_back(p);
/*			
			std::cout << e.p1_.x_ << ',' << e.p1_.y_ << std::endl;
			std::cout << e.p2_.x_ << ',' << e.p2_.y_ << std::endl;*/
		}
		
		for(const auto& point : path_vec)
		{
			geometry_msgs::Point p;
			p.x = point.x_;
			p.y = point.y_;
			p.z = 0;
			line_strip.points.push_back(p);
		}
// 		
// 		for(uint32_t i = 0; i < 100; ++i)
// 		{
// 			float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
// 			float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
// 			
// 			geometry_msgs::Point p;
// 			p.x = (int32_t)i - 50;
// 			p.y = y;
// 			p.z = z;
// 			
// 			points.points.push_back(p);
// 			line_strip.points.push_back(p);
// 			
// 			line_list.points.push_back(p);
// 			
// 			p.z += 1.0;
// 			line_list.points.push_back(p);
// 		}
// 		
		marker_pub.publish(points);

		marker_pub.publish(line_list);
		marker_pub.publish(line_list2);
		marker_pub.publish(line_strip);
// 		
		r.sleep();
// 		
// 		f += 0.04;
	}
}