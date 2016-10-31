#include"main.h"
#include "corner_point.h"

#define FRAME_SUM 200


//cv::Point2f head =cv::Point2f(1043,405);//0帧
//cv::Point2f head_point = cv::Point2f(1033,390);

//cv::Point2f head =cv::Point2f(1040,371);
//cv::Point2f head_point = cv::Point2f(1039,338);//23帧头顶
cv::Point2f head = cv::Point2f(796,410);//out_test 0帧
cv::Point2f head_point = cv::Point2f(796,359);//out_test 0帧头顶
//cv::Point2f head = cv::Point2f(881,418);//out_test 250帧
//cv::Point2f head = cv::Point2f(178,372);//out_test 400帧
//cv::Point2f head_point = cv::Point2f(200,372);//out_test 400帧头顶
//cv::Point2f head = cv::Point2f(818,307);//out_test 820帧
//cv::Point2f head_point = cv::Point2f(824,256);//out_test 820帧头顶
//cv::Point2f head2 = cv::Point2f(815,324);
//cv::Point2f head = cv::Point2f(281,221);//out_test1
//cv::Point2f head_point = cv::Point2f(281,231);//out_test1 0帧头顶
//cv::Point2f head = cv::Point2f(244,203);//out_test3
//cv::Point2f head_point = cv::Point2f(244,208);//out_test3 0帧头顶


//cv::Point2f head =cv::Point2f(472,228);
//cv::Point2f head_point = cv::Point2f(472,188);

//cv::Point2f head = cv::Point2f(903,474);//lab_test
cv::Mat gray_prev ,gray,gray_copy;
//int fxy_max , fxy_min;
cv::Mat cluster;

cv::Mat pre_frame,copy_frame , canny_frame , b_frame;



int main()
{
    //聚类测试
//    cv::Mat temp(2000,2000,CV_8UC3,cv::Scalar(255,255,255));//cluster
//    cv::line(temp,cv::Point(2000,800),cv::Point(0,800),cv::Scalar(100));
//    cv::line(temp,cv::Point(800,2000),cv::Point(800,0),cv::Scalar(100));
    cv::Mat temp(2000,2000,CV_8UC3,cv::Scalar(255,255,255));//cluster
    cv::line(temp,cv::Point(2000,800),cv::Point(0,800),cv::Scalar(100));
    cv::line(temp,cv::Point(800,2000),cv::Point(800,0),cv::Scalar(100));


    //读取视频
    cv::VideoCapture capture("F:/processed/out_test.avi");
    if(!capture.isOpened()) std::cout<< "capture is not opened"<<std::endl;
    cv::Mat frame ;
//    capture >> frame;
    for(int test_num = 0 ; test_num  < 1 ; test_num++)//foreigner
    {
        capture >>  frame;
    }
    cv::Mat first_gray;
    cvtColor(frame, first_gray, CV_BGR2GRAY);
//    cv::VideoWriter writer("v50_foreigner_23frame.avi", CV_FOURCC('M', 'J', 'P', 'G'),1.0, cv::Size(frame.cols, frame.rows),false); //写出结果视频
//    cv::VideoWriter writer_cluster("v50_cluster_gradual.avi", CV_FOURCC('M', 'J', 'P', 'G'),1.0, cv::Size(2000, 2000)); //写出结果视频
    cv::Mat cornerStrength;
    cv::cornerHarris(first_gray, cornerStrength, 2, 3, 0.01);


    //对灰度图进行阈值操作，得到二值图并显示
    cv::Mat harrisCorner;
    cv::threshold(cornerStrength, harrisCorner, 0.0005/*0.000008*//*0.00000005*/, 255,cv::THRESH_BINARY);
//    cv::imshow("harris",harrisCorner);
//    cv::waitKey(0);

    std::vector<cv::Point2f> corner_points;
    for(int i =0 ; i< frame.cols ; i++)
    {
        for(int j = 0 ; j< frame.rows ; j++)
        {
            if((unsigned int)harrisCorner.at<float>(j,i) == 255)
            {
                corner_points.push_back(cv::Point2f(i,j));
            }
        }
    }

    cv::imshow("harris",harrisCorner);
    cv::waitKey(0);


    std::cout<< "corner num : "<<(int)corner_points.size()<<std::endl;

    //筛选头点
    std::vector<cv::Point2f> head_point_track; //光流
    std::vector<Corner_point> v_corner_points;
    for(long i =0; i< (long)corner_points.size();i++)
    {
        if( (corner_points[i].x - head.x)*(corner_points[i].x - head.x) + (corner_points[i].y - head.y)*(corner_points[i].y - head.y) <= 900)
        {
            head_point_track.push_back(corner_points[i]);
            Corner_point temp_point(corner_points[i]);
            v_corner_points.push_back(temp_point);
        }
    }
//    for(long i =0; i< (long)corner_points.size();i++)
//    {
//        if( (corner_points[i].x - head2.x)*(corner_points[i].x - head2.x) + (corner_points[i].y - head2.y)*(corner_points[i].y - head2.y) <= 200)
//        {
//            head_point_track.push_back(corner_points[i]);
//            Corner_point temp_point(corner_points[i]);
//            v_corner_points.push_back(temp_point);
//        }
//    }
    //跟踪

    char framename[100];
    int cluster_num;
    for(int framenum = 0 ; !frame.empty()&& framenum < 400; framenum++, capture >> frame)
    {
        temp.copyTo(cluster);//cluster
        cluster_num = tracking(frame,v_corner_points,head_point);//Corner_point
//        tracking(frame,head_point_track,head_point);//original
//        draw_points(frame,head_point_track,head_point);//original
        std::cout<<"corner_point num : "<<(int)v_corner_points.size()<<std::endl;
        std::cout<<"frame_num : "<< framenum <<std::endl;


//        writer << gray_copy; //视频
//        writer_cluster << cluster;//cluster视频


        sprintf(framename,"out_test3_cluster/%d_%d.jpg",framenum,cluster_num);//cluster
        cv::imwrite(framename,cluster);//cluster
        sprintf(framename,"out_test3/%d.jpg",framenum);
        cv::imwrite(framename,gray_copy/*frame*/);//Corner_point
//        cv::imwrite(framename,/*gray_copy*/frame);//original

        cv::imshow("cluster",cluster);//cluster
        cv::imshow("frame",gray_copy/*frame*/);//Corner_point
//        cv::imshow("frame",/*gray_copy*/frame);//original
        cv::waitKey(200);
    }


//    std::cout << "fxy_max : " <<fxy_max<<std::endl<<"fxy_min : "<<fxy_min;


    return 0;
}



