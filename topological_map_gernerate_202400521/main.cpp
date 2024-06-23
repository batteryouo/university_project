#include <algorithm>
#include <iostream>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "initial_map.hpp"
#include "path.hpp"

#define ADDTION_POINT_INTERVAL 30 
#define RESOLUTION 0.04

using namespace cv;
using namespace std;

int vertex_size = 0;
int test_p = 1;
Voronoi_vertex full_vex_edge[2000], roi_vex_edge[2000], temp_vex_edge[800], mix_vex_edge[2000], little_vex_edge[800];

bool simulation = true;
// string map_path = "./map/lab_0531_2.pgm";
// string map_path = "./map/image.png";
string map_path = "./map/test_image.png";
//robot postition
Point2f robot_position;

Point2f goal_position = Point2f(100, 100);
// Point2f goal_position = Point2f(177, 45);
Point2f left_point;
int is_on_path = 0;
int goal_num;
vector<int> path_final_j, path_final_j_inv;
vector<int> cut_path_glob;
vector<int> fixed_path;
int fixed_path_cnt = 0;
Point2f aft_robot_pos, cur_robot_pos, r_next_pos;
int obs_px = 0, last_obs_px = 9, obs_py = 0, last_obs_py = 9, cnt = 1;

namespace myNameSpace{
	void myImshow(string windows_name, Mat img){
		if (simulation){
			namedWindow(windows_name, 0);
			imshow(windows_name, img);
		}
	}

	void myWaitKey(int time = 0){
		if (simulation){
			waitKey(time);
		}
	}
}

int main(int argc, char **argv){
	double start_time, end_time;

	int channel_num = 0;
	uint8_t* pixelPtr;
	Scalar_<uint8_t> bgrPixel;

	start_time = clock();
	Mat map_gray_original, map_gray_process, map_rgb_process, delaunay_img, voronoi_img;
		
	//initial robot position
	//row major             (column, row)
	robot_position = Point2f(1500, 1000);
	// robot_position = Point2f(1500, 100);
	map_gray_original = imread(map_path, 0);
	map_gray_original.copyTo(map_gray_process);
	cvtColor(map_gray_process, map_rgb_process, COLOR_GRAY2BGR);
	// circle(map_rgb_process, robot_position, 10, Scalar(255, 0, 0), FILLED);
	// pixelPtr = (uint8_t*)map_gray_original.data;
	// channel_num = map_gray_original.channels();
	// bgrPixel.val[0] = pixelPtr[1*map_gray_original.cols*channel_num + 1*channel_num + 0];
	// bgrPixel.val[1] = pixelPtr[1*map_gray_original.cols*channel_num + 1*channel_num + 1];
	// bgrPixel.val[2] = pixelPtr[1*map_gray_original.cols*channel_num + 1*channel_num + 2];
	// cout << "color: [" << (int)bgrPixel.val[0] << " " << (int)bgrPixel.val[1] << " " << (int)bgrPixel.val[2] << "]" << endl;
	end_time = clock();
	cout << "cost time of read map: " << (end_time - start_time) / CLOCKS_PER_SEC << "sec" << endl;

	start_time = clock();
	myNameSpace::myImshow("map", map_gray_original);

	InitialMap initial_map(map_gray_original, ADDTION_POINT_INTERVAL);
	// Preprocessing map
	initial_map.mapPreprocessing();

	map_gray_process = initial_map.getMap();
	initial_map.grayMap2RGBMap();
	myNameSpace::myImshow("map_process", map_gray_process);
	end_time = clock();
	cout << "cost time of preprocessing image: " << (end_time - start_time) / CLOCKS_PER_SEC << "sec" << endl;

	// get obstacle points	
	start_time = clock();	
	initial_map.addtionPoint();
	map_rgb_process = initial_map.getRGBMap();

	myNameSpace::myImshow("obstacle point", map_rgb_process);
	end_time = clock();
	myNameSpace::myWaitKey(0);	
	cout << "cost time of get obstacle point: " << (end_time-start_time)/CLOCKS_PER_SEC << "sec" << endl;
	initial_map.grayMap2RGBMap();
	
	// draw delaunay
	start_time = clock();
	// reset the rgb map of PathPlanning class
	// initial_map.setPathRGBImg(map_rgb_process);
	// initial_map.delaunay();
	// get delaunay image (getPathRGBMap is to get the rgb image from path planning)
	// at this time the rgb image is delaunay image
	// delaunay_img = initial_map.getPathRGBMap();
	// myNameSpace::myImshow("delaunay", delaunay_img);
	initial_map.setPathRGBImg(map_rgb_process);
	initial_map.voronoi();
	voronoi_img = initial_map.getPathRGBMap();
	myNameSpace::myImshow("voronoi", voronoi_img);
	end_time = clock();
	cout << "cost time of voronoi: " << (end_time-start_time)/CLOCKS_PER_SEC << "sec" << endl;

	initial_map.setPathRGBImg(map_rgb_process);
	int num_cnt = 1, num_edge_cnt = 1;
	if (mix_vex_edge[0].vex_size == 0) {
		// do not detect obstacle at beginning
		obs_px = 3;
		obs_py = 3;
		for (int i = 0; i < full_vex_edge[0].vex_size; i++) {

			mix_vex_edge[num_cnt - 1].x = full_vex_edge[i].x;
			mix_vex_edge[num_cnt - 1].y = full_vex_edge[i].y;
			mix_vex_edge[num_cnt - 1].num = num_cnt;
			mix_vex_edge[num_cnt - 1].num = full_vex_edge[i].num;
			num_cnt++;


		}
		mix_vex_edge[0].vex_size = num_cnt - 1;

		vector<Point2f> after_save;
		vector<int> after_save_num;

		for (int i = 0; i < full_vex_edge[0].edge_size; i++) {

			mix_vex_edge[num_edge_cnt - 1].edge_link_pos[0] = full_vex_edge[i].edge_link_pos[0];
			mix_vex_edge[num_edge_cnt - 1].edge_link_pos[1] = full_vex_edge[i].edge_link_pos[1];

			mix_vex_edge[num_edge_cnt - 1].edge_link_pos[2] = full_vex_edge[i].edge_link_pos[2];
			mix_vex_edge[num_edge_cnt - 1].edge_link_pos[3] = full_vex_edge[i].edge_link_pos[3];

			mix_vex_edge[num_edge_cnt - 1].edge_link_num[0] = full_vex_edge[i].edge_link_num[0];
			mix_vex_edge[num_edge_cnt - 1].edge_link_num[1] = full_vex_edge[i].edge_link_num[1];

			num_edge_cnt++;

		}
		mix_vex_edge[0].edge_size = num_edge_cnt - 1;
	}
	Mat tmp_img1 = map_rgb_process;
	initial_map.replan_astar(mix_vex_edge, tmp_img1);

	myNameSpace::myWaitKey(0);
	
	return 0;
}