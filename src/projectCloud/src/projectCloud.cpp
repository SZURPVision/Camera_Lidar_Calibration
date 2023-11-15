#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "common.h"
#include "result_verify.h"
#include "CustomMsg.h"
float max_depth = 60;
float min_depth = 1;
vector<livox_ros_driver::CustomMsg> lidar_datas; 
int threshold_lidar = 500000;  // number of cloud point on the photo
string input_bag_path, input_photo_path, output_path, intrinsic_path, extrinsic_path;


void loadPointcloudFromROSBag(const string& bag_path) {
    ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg"));  // message title
    rosbag::View view(bag, rosbag::TypeQuery(types));

    for (const rosbag::MessageInstance& m : view) {
        livox_ros_driver::CustomMsg livoxCloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
        lidar_datas.push_back(livoxCloud);
        if (lidar_datas.size() > (threshold_lidar/24000 + 1)) {
            break;
        }
    }
}

// set the color by distance to the cloud
void getColor(int &result_r, int &result_g, int &result_b, float cur_depth) {
    float scale = (max_depth - min_depth)/10;
    if (cur_depth < min_depth) {
        result_r = 0;
        result_g = 0;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale) {
        result_r = 0;
        result_g = int((cur_depth - min_depth) / scale * 255) & 0xff;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale*2) {
        result_r = 0;
        result_g = 0xff;
        result_b = (0xff - int((cur_depth - min_depth - scale) / scale * 255)) & 0xff;
    }
    else if (cur_depth < min_depth + scale*4) {
        result_r = int((cur_depth - min_depth - scale*2) / scale * 255) & 0xff;
        result_g = 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*7) {
        result_r = 0xff;
        result_g = (0xff - int((cur_depth - min_depth - scale*4) / scale * 255)) & 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*10) {
        result_r = 0xff;
        result_g = 0;
        result_b = int((cur_depth - min_depth - scale*7) / scale * 255) & 0xff;
    }
    else {
        result_r = 0xff;
        result_g = 0;
        result_b = 0xff;
    }

}



void Get_param(ros::NodeHandle& nh, Eigen::Matrix3d &cameraMatrix, Eigen::VectorXd &distCoeffs, Eigen::Matrix4d &camera2LidarMatrix)
{
    cameraMatrix << 4161.1191, 0, 882.8221,
                    0, 4183.0083, 552.3232,
                    0, 0, 1;

    distCoeffs.resize(5);
    distCoeffs << -0.194475, 0.798209, 0.002118, -0.004708, 0;

    nh.getParam("/intrinsic_bar_val/r1_val", camera2LidarMatrix(0, 0));
    nh.getParam("/intrinsic_bar_val/r2_val", camera2LidarMatrix(0, 1));
    nh.getParam("/intrinsic_bar_val/r3_val", camera2LidarMatrix(0, 2));
    nh.getParam("/intrinsic_bar_val/r4_val", camera2LidarMatrix(1, 0));
    nh.getParam("/intrinsic_bar_val/r5_val", camera2LidarMatrix(1, 1));
    nh.getParam("/intrinsic_bar_val/r6_val", camera2LidarMatrix(1, 2));
    nh.getParam("/intrinsic_bar_val/r7_val", camera2LidarMatrix(2, 0));
    nh.getParam("/intrinsic_bar_val/r8_val", camera2LidarMatrix(2, 1));
    nh.getParam("/intrinsic_bar_val/r9_val", camera2LidarMatrix(2, 2));
    nh.getParam("/intrinsic_bar_val/t1_val", camera2LidarMatrix(0, 3));
    nh.getParam("/intrinsic_bar_val/t2_val", camera2LidarMatrix(1, 3));
    nh.getParam("/intrinsic_bar_val/t3_val", camera2LidarMatrix(2, 3));

}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"projectCloud");
    ros::NodeHandle nh("~");
    std::string img_path = "/home/rp/DYH/lidar2camera_ws/src/projectCloud/src/0.bmp";
    std::string input_bag_path = "/home/rp/DYH/lidar2camera_ws/src/projectCloud/src/0.bag";
    cv::Mat img = cv::imread(img_path);
    cv::Mat img_ = img.clone();
    if(img.empty())
    {
        ROS_INFO("IMG EMPTY!\n");
        // return 0;
    }
    else
    {
        cv::namedWindow("img", cv::WINDOW_NORMAL);
    }
    // cv::imshow("img",img);
    // cv::waitKey(0);
    loadPointcloudFromROSBag(input_bag_path);
    Eigen::Matrix3d cameraMatrix = Eigen::Matrix3d::Identity();
    Eigen::VectorXd distCoeffs = Eigen::VectorXd(5);
    Eigen::Matrix4d camera2LidarMatrix = Eigen::Matrix4d::Identity();        
    // cv::Mat camera2LidarMatrix = cv::Mat::eye(4,4, CV_64F);
    // Get_param(nh, cameraMatrix, distCoeffs, camera2LidarMatrix);
    ROS_INFO("Start to project the lidar points!\n");
    std::vector<cv::Point> points_to_draw;
    std::vector<cv::Scalar> scala;
    while(ros::ok())
    {
        img = img_.clone();
        // cv::waitKey(1);
        // int key = cv::waitKey(1);
        points_to_draw.clear();
        scala.clear();
        Get_param(nh, cameraMatrix, distCoeffs, camera2LidarMatrix);
        // img = img_.clone();
        Eigen::Matrix<double, 3, 4> camera2LidarMatrix_transfrom ;
        camera2LidarMatrix_transfrom = camera2LidarMatrix.block<3,4>(0,0); 
        std::stringstream ss;
        ss << camera2LidarMatrix;
        ROS_INFO("Camera_Lidar_Matrix:\n%s",ss.str().c_str());
        float x, y, z;
        float theoryUV[2] = {0, 0};
        int myCount = 0;
        cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1); //内参矩阵
        K.at<float>(0, 0) = 4161.1191;
        K.at<float>(1, 1) = 4183.0083;
        K.at<float>(0, 2) = 882.8221;
        K.at<float>(1, 2) = 552.3232;
        cv::Mat distort_coeffs = cv::Mat::zeros(1, 5, CV_32FC1); //畸变系数矩阵 顺序是[k1, k2, p1, p2, k3]
        distort_coeffs.at<float>(0, 0) =  -0.194475;
        distort_coeffs.at<float>(0, 1) = 0.798209;
        distort_coeffs.at<float>(0, 2) = 0.002118;
        distort_coeffs.at<float>(0, 3) = -0.004708;
        int rows = img.rows, cols = img.cols;
        cv::Mat image_undistort2 = cv::Mat(rows, cols, CV_8UC3);  // 方法2 OpenCV去畸变以后的图
        cv::undistort(img, image_undistort2, K, distort_coeffs); //去畸变
        for(unsigned int i = 0; i < lidar_datas.size(); ++i)
        {
            for(unsigned int j = 0; j < lidar_datas[i].point_num; ++j)
            {
                x = lidar_datas[i].points[j].x;
                y = lidar_datas[i].points[j].y;
                z = lidar_datas[i].points[j].z;
                Eigen::Vector4d points(x, y, z, 1.0);
                Eigen::Vector3d result = cameraMatrix * camera2LidarMatrix_transfrom * points;
                ROS_INFO("Project point {x:%f , y:%f , z:%f}\n", result(0),result(1),result(2));
                // 计算在像素平面的坐标值
                int u = float(result(0)/result(2) + 0.5);
                int v = float(result(1)/result(2) + 0.5);
                cv::Point p(u, v);
                int r,g,b;
                getColor(r, g, b, x);
                scala.push_back(cv::Scalar(b,g,r));
                // cv::circle(img, p, 3, cv::Scalar(b, g, r), -1);
                // cv::imshow("img",img);
                // cv::waitKey(1);
                 points_to_draw.push_back(cv::Point(u, v));
                 
                ++myCount;
                if (myCount > threshold_lidar) {
                    break;
                }
            }
            if (myCount > threshold_lidar) {
                break;
                }
        }
        for(int i = 0;i < points_to_draw.size();i++)
        {
             cv::circle(image_undistort2, points_to_draw[i], 2, scala[i], -1);
        }


        // 在渲染所有点后显示图像
        cv::imshow("img", image_undistort2);
        cv::waitKey(10);
        // std::cout<< camera2LidarMatrix<<"--------------"<<std::endl;
    }
        
}