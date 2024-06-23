#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#ifndef PATH_H
#define PATH_H

class Voronoi_vertex{
	public:
		Voronoi_vertex();
		~Voronoi_vertex();
		// x coordinate of vertex 
		float x;
		// y coordinate of vertex
		float y;
		// number of point( original ID )
		int origin;
		//number of num_match ( new ID -> start from 1)
		int num;
		// the new vertex IDs (a pair -> 2 vertexes) of this edge
		int edge_link_num[2];

		// the original vertex IDs ( a pair -> 2 vertexes) of this edge 
		int edge_link_origin[2];

		// edge_link_pos[0] -> start_point.x of this edge
		// edge_link_pos[1] -> start_point.y of this edge
		// edge_link_pos[2] -> end_point.x of this edge
		// edge_link_pos[3] -> end_point.y of this edge
		float edge_link_pos[4];

		// total vextex amount
		int vex_size;

		// total edge amount
		int edge_size;
	private:
};

class PathPlanning{
	public:
		PathPlanning(cv::Mat &img);
		~PathPlanning();
		void voronoi();
		void delaunay();
		void setPathRGBImg(cv::Mat &inputImg);
		void setPathImg(cv::Mat &inputImg);
		void aStar();
		cv::Mat getPathRGBMap();
		void replan_astar(Voronoi_vertex *vex_edge, cv::Mat &outputImg);
	protected:
		cv::Mat path_img;
		cv::Mat path_img_rgb;
		cv::Mat cost_map;
		std::vector<cv::Point2f> points;
		cv::Rect rect;
		cv::Subdiv2D subdiv;
		std::vector<cv::Point2f> voronoi_points;
		
	private:
		bool isThroughObstacle(cv::Point a, cv::Point b, int thickness = 1);
};



extern Voronoi_vertex full_vex_edge[2000], roi_vex_edge[2000], temp_vex_edge[800], mix_vex_edge[2000], little_vex_edge[800];

#endif