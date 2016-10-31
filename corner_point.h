#ifndef CORNER_POINT_H
#define CORNER_POINT_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <stdio.h>
#include <stdlib.h>

class Corner_point
{
public:
    int r_o_w ;//>0记为可使用为约束 =0记为暂不可用 <0记为不可用 初试置为1
    int wrong_track_num; //标记暂不可用的次数 初始置为0
    cv::Point2f point;
    cv::Point2f dispalcement;

    //寻找簇中心
    cv::Point2f cluster_dpm;//用于计算簇的位移
    static double radius;//半径
    int density;//密度
    double min_distance;//到较大密度点的最小距离
    bool if_center;//是否密度中心
    int get_density(std::vector<Corner_point> Points);
    double get_mdistance(std::vector<Corner_point> Points);

    //DBSCAN
    int cluster_mark;//聚类的标记
    int db_density;//DBSCAN所用密度
    static double core_dis;
    int get_db_density(std::vector<Corner_point> Points);



    Corner_point(cv::Point2f get_point);
    void move(cv::Point2f fxy);
    void get(cv::Point2f get_point);



};

void find_center(std::vector<Corner_point> & Corner_points, std::vector<int> &cluster_centre);

bool if_inside(Corner_point center,Corner_point test_point);

int get_cluster(std::vector<Corner_point> & Corner_points, std::vector<int> cluster_centre);

//void get_meanvec(std::vector<Corner_point> & Corner_points,int cluster_num );

cv::Point2f get_mean_vector(std::vector<Corner_point> & Corner_points);//temp

#endif // CORNER_POINT_H
