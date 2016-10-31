#include"main.h"

extern cv::Mat gray_prev ,gray,gray_copy;
//extern int fxy_max , fxy_min;
extern cv::Mat cluster;

extern cv::Mat pre_frame,copy_frame, canny_frame,b_frame;


void draw_arrow(cv::Mat & frame , cv::Point2f fxy , cv::Point2f tail , cv::Scalar arrow_color)
{
    const float PI = 3.14159;
    cv::Point2f arrow_head;
    fxy = fxy*10;
    //画线
    arrow_head = tail + fxy;
    cv::line(frame,tail , arrow_head ,arrow_color );
    cv::circle(frame,tail,1,arrow_color);

    //画箭头
    cv::Point2f arrow;
    float len = sqrtf((fxy.x*fxy.x)+(fxy.y*fxy.y))/3; //箭头长度
    float arrow_angle = 30;
    float angle = atan2((double)(tail.y - arrow_head.y), (double)(tail.x - arrow_head.x));  //计算箭头指向角度

    arrow.x = arrow_head.x + len * cos(angle + PI *  arrow_angle/ 180);   //箭头夹角15度
    arrow.y = arrow_head.y + len * sin(angle + PI * arrow_angle/ 180);
    cv::line(frame, arrow_head, arrow, arrow_color);
    arrow.x = arrow_head.x + len * cos(angle - PI * arrow_angle/ 180);
    arrow.y = arrow_head.y + len * sin(angle - PI * arrow_angle/ 180);
    cv::line(frame, arrow_head, arrow, arrow_color);
}



int tracking(cv::Mat & frame , std::vector<cv::Point2f> & head_point_track , cv::Point2f & head_point) //光流追踪
{
//    static cv::Mat gray_prev;	// 先前图片
    cv::Mat flow;
    cvtColor(frame, gray, CV_BGR2GRAY);

    if (gray_prev.empty())
    {
        frame.copyTo(pre_frame);
        gray.copyTo(gray_prev);
        gray.copyTo(gray_copy);//用于arrow
        return 0;
    }
    gray_prev.copyTo(gray_copy);//用于arrow


    //跟踪
    cv::calcOpticalFlowFarneback(gray_prev,gray,flow,0.5,3,15,3,5,1.2,0);
    for(int i=0;i<(int)head_point_track.size();i++)
    {
        if((head_point_track[i].x < frame.cols)&&(head_point_track[i].y < frame.rows)&&(head_point_track[i].x > 0)&&(head_point_track[i].y > 0))
        {
            cv::Point2f fxy = flow.at<cv::Point2f>(head_point_track[i].y, head_point_track[i].x);
//            draw_arrow(gray_copy,fxy,head_point_track[i],cv::Scalar(255));//arrow
            head_point_track[i] = head_point_track[i]+fxy;

//            fxy = 10 * (fxy+cv::Point2f(16,16));//cluster
////            std::cout << "color : "<< (unsigned int )cluster.at<uchar>(fxy)<<std::endl;

//            if((unsigned int )cluster.at<uchar>(fxy) > 0)
//                cluster.at<uchar>(fxy) -= 51 ;
////            std::cout << "color : "<< (unsigned int )cluster.at<uchar>(fxy)<<std::endl;

        }


    }

    cv::Point2f fxy = flow.at<cv::Point2f>(head_point.y, head_point.x);
    draw_arrow(gray_copy,fxy,head_point,cv::Scalar(255));//arrow
    head_point = head_point+fxy;
    fxy = 10 * (fxy+cv::Point2f(16,16));//cluster
    cv::circle(cluster,fxy,1,cv::Scalar(100),-1);//cluster
    cluster.at<uchar>(fxy) = 0;


    frame.copyTo(pre_frame);
    swap(gray_prev, gray);
    return 0;
}

int tracking(cv::Mat & frame , std::vector<Corner_point> & head_point_track , cv::Point2f & head_point)
{
    cv::Mat flow;
    cvtColor(frame, gray, CV_BGR2GRAY);

    if (gray_prev.empty())
    {
        frame.copyTo(pre_frame);
        gray.copyTo(gray_prev);
        gray.copyTo(gray_copy);//用于arrow
        return 0;
    }
    gray_prev.copyTo(gray_copy);//用于arrow
    frame.copyTo(copy_frame);

    //跟踪
    cv::calcOpticalFlowFarneback(gray_prev,gray,flow,0.5,3,15,3,5,1.2,0);
    for(int i=0;i<(int)head_point_track.size();i++)
    {
        if((head_point_track[i].point.x < frame.cols)&&(head_point_track[i].point.y < frame.rows)&&(head_point_track[i].point.x > 0)&&(head_point_track[i].point.y > 0))
        {
            cv::Point2f fxy = flow.at<cv::Point2f>(head_point_track[i].point.y, head_point_track[i].point.x);
            draw_arrow(gray_copy,fxy,head_point_track[i].point,cv::Scalar(255));//arrow
            head_point_track[i].move(fxy);
        }
    }

    std::cout<<"head_point_track num : "<<head_point_track.size()<<std::endl;
    std::cout<<"point_num : "<< head_point_track.size()<<std::endl;
    std::vector<int> cluster_center;
    find_center(head_point_track,cluster_center);
    int cluster_num;
    cluster_num = get_cluster(head_point_track,cluster_center);
    std::cout<<"cluster_num : "<<cluster_num<<std::endl;
    for(int i=0;i<(int)head_point_track.size();i++)
    {
        cv::Point2f fxy = head_point_track[i].cluster_dpm + cv::Point2f(800,800);
        if(head_point_track[i].cluster_mark ==0)
        {
            cluster.at<cv::Vec3b>(fxy) = cv::Vec3b(255,0,0);
        }
        else if(head_point_track[i].cluster_mark ==1)
        {
            cluster.at<cv::Vec3b>(fxy) = cv::Vec3b(0,255,0);
        }
        else if(head_point_track[i].cluster_mark ==2)
        {
            cluster.at<cv::Vec3b>(fxy) = cv::Vec3b(0,0,255);
        }
        else
        {
            cluster.at<cv::Vec3b>(fxy) = cv::Vec3b(0,255,255);
        }

    }


    cv::bilateralFilter( copy_frame, b_frame, 5, 100, 25/2 );
//    cv::imshow("b",b);
    cv::Canny(b_frame,canny_frame,10,30);
//    cv::imshow("out",out);
//    cv::waitKey(0);
    cv::Point2f mean_vector;
    mean_vector = get_mean_vector(head_point_track);
    cv::Point2f temp_headpoint =head_point+mean_vector;
    cv::Mat temp_canny = canny_frame(cv::Rect(temp_headpoint.x -50,temp_headpoint.y - 50,100,100));
    cv::Mat weight_frame;
    cvtColor(temp_canny, weight_frame, CV_GRAY2BGR);
    cv::line(weight_frame,cv::Point2f(50-20*mean_vector.x,50-20*mean_vector.y),cv::Point2f(50/*+20*mean_vector.x*/,50/*+20*mean_vector.y*/),cv::Scalar(0,0,255));
    cv::Mat temp_frame = copy_frame(cv::Rect(temp_headpoint.x -50,temp_headpoint.y - 50,100,100));
    addWeighted(temp_frame,0.5,weight_frame,0.5,0.0,temp_frame);
    cv::imshow("canny_test",copy_frame);
    cv::waitKey(0);


    cv::Point2f fxy = flow.at<cv::Point2f>(head_point.y, head_point.x);
    draw_arrow(gray_copy,fxy,head_point,cv::Scalar(255));//arrow
    head_point = head_point+fxy;
    fxy = 10 * (fxy+cv::Point2f(16,16));//cluster
    //cv::circle(cluster,fxy,1,cv::Scalar(100),-1);//cluster
    cluster.at<cv::Vec3b>(fxy) = cv::Vec3b(0,0,0);


    frame.copyTo(pre_frame);
    swap(gray_prev, gray);
    return cluster_num;
}


void draw_points(cv::Mat & frame  , std::vector<cv::Point2f> & head_point_track, cv::Point2f head_point )
{
    for(int i = 0 ; i < (int)head_point_track.size() ; i++)
    {
        cv::circle(frame,head_point_track[i],1,cv::Scalar(0,255,0),-1);
    }

    cv::circle(frame,head_point,1,cv::Scalar(255,0,0),-1);
}

void draw_points_gray(cv::Mat & gray_frame  , std::vector<cv::Point2f>  head_point_track, cv::Point2f head_point)
{
    for(int i = 0 ; i < (int)head_point_track.size() ; i++)
    {
        if((head_point_track[i].x < gray_frame.cols)&&(head_point_track[i].y < gray_frame.rows)&&(head_point_track[i].x > 0)&&(head_point_track[i].y > 0))
        cv::circle(gray_frame,head_point_track[i],1,cv::Scalar(255),-1);
    }

    cv::circle(gray_frame,head_point,1,cv::Scalar(255),-1);
}

