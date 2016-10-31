#ifndef MAIN_H
#define MAIN_H

#include"corner_point.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>





void draw_arrow(cv::Mat & frame , cv::Point2f fxy , cv::Point2f tail , cv::Scalar arrow_color);

void get_every_density(std::vector<Corner_point> & Corner_points);

void get_min_dis(std::vector<Corner_point> & Corner_points);

int tracking(cv::Mat & frame , std::vector<cv::Point2f> & head_point_track , cv::Point2f & head_point); //光流追踪

int tracking(cv::Mat & frame , std::vector<Corner_point> & head_point_track , cv::Point2f & head_point);

void draw_points(cv::Mat & frame  , std::vector<cv::Point2f> & head_point_track, cv::Point2f head_point );

void draw_points_gray(cv::Mat & gray_frame  , std::vector<cv::Point2f>  head_point_track, cv::Point2f head_point);

void read_corner_points(std::ifstream & ifile ,std::vector<cv::Point2f> & corner_points);



#endif // MAIN_H

