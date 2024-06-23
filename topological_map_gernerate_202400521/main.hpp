#include <vector>
#include "opencv2/core.hpp"

#ifndef MAIN_H
#define MAIN_H

#define ADDTION_POINT_INTERVAL 30 
#define RESOLUTION 0.04

extern int vertex_size;
extern int test_p;
extern bool simulation;
extern std::string map_path;
//robot postition
extern cv::Point2f robot_position;
extern cv::Point2f goal_position;

extern cv::Point2f left_point;
extern int is_on_path;
extern int goal_num;

extern cv::Point2f aft_robot_pos, cur_robot_pos, r_next_pos;
extern std::vector<int> path_final_j, path_final_j_inv;
extern std::vector<int> cut_path_glob;
extern std::vector<int> fixed_path;
extern int fixed_path_cnt;
extern int obs_px, last_obs_px, obs_py, last_obs_py, cnt;
namespace myNameSpace{
	void myImshow(std::string windows_name, cv::Mat img);

	void myWaitKey(int time = 0);
}
#endif