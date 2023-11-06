#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/core.hpp"
#include "ros/ros.h"
using namespace cv;

class bar
{
public:
    bar(std::string name, ros::NodeHandle& nh)
        : nh_(nh) 
    {
        this->r1_real_val = 0;
        this->r2_real_val = 0;
        this->r3_real_val = 0;
        this->r4_real_val = 0;
        this->r5_real_val = 0;
        this->r6_real_val = 0;
        this->r7_real_val = 0;
        this->r8_real_val = 0;
        this->r9_real_val = 0;
        this->t1_real_val = 0;
        this->t2_real_val = 0;
        this->t3_real_val = 0;
        this->r1_val = 0;
        this->r2_val = 0;
        this->r3_val = 0;
        this->r4_val = 0;
        this->r5_val = 0;
        this->r6_val = 0;
        this->r7_val = 0;
        this->r8_val = 0;
        this->r9_val = 0;
        this->t1_val = 0;
        this->t2_val = 0;
        this->t3_val = 0;
        nh_.param("/intrinsic_bar_val/r1_val",r1_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r2_val",r2_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r3_val",r3_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r4_val",r4_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r5_val",r5_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r6_val",r6_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r7_val",r7_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r8_val",r8_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/r9_val",r9_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/t1_val",t1_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/t2_val",t2_real_val, 0.0);
        nh_.param("/intrinsic_bar_val/t3_val",t3_real_val, 0.0);
        namedWindow(name,WINDOW_NORMAL);
        create_bar();  
        // imshow(name)
    }
    ~bar(){
        saveYamlFile("/home/rp/DYH/lidar2camera_ws/src/track_bar/config/intrinsic.yaml");
    }
    inline void create_bar()
    {
        std::cout<<"yes";
        createTrackbar("r1_val","bar",&r1_val,2000,&bar::on_callback,this);
        createTrackbar("r2_val","bar",&r2_val,2000,&bar::on_callback,this);
        createTrackbar("r3_val","bar",&r3_val,2000,&bar::on_callback,this);
        createTrackbar("r4_val","bar",&r4_val,2000,&bar::on_callback,this);
        createTrackbar("r5_val","bar",&r5_val,2000,&bar::on_callback,this);
        createTrackbar("r6_val","bar",&r6_val,2000,&bar::on_callback,this);
        createTrackbar("r7_val","bar",&r7_val,2000,&bar::on_callback,this);
        createTrackbar("r8_val","bar",&r8_val,2000,&bar::on_callback,this);
        createTrackbar("r9_val","bar",&r9_val,2000,&bar::on_callback,this);
        createTrackbar("t1_val","bar",&t1_val,2000,&bar::on_callback,this);
        createTrackbar("t2_val","bar",&t2_val,2000,&bar::on_callback,this);
        createTrackbar("t3_val","bar",&t3_val,2000,&bar::on_callback,this);
    }
private:
    int r1_val, r2_val, r3_val, r4_val, r5_val, r6_val, r7_val, r8_val,r9_val;
    int t1_val, t2_val, t3_val;
    double r1_real_val, r2_real_val, r3_real_val, r4_real_val, r5_real_val, r6_real_val, r7_real_val, r8_real_val, r9_real_val;
    double t1_real_val, t2_real_val, t3_real_val;
    ros::NodeHandle nh_;
    static void on_callback(int value, void* userdata)
    {
        bar* self = static_cast<bar*>(userdata);
        // self->on_callback(value);
        self->r1_real_val = self->r1_val / 1000.0 * 1.0 - 1;
        self->r2_real_val = self->r2_val / 1000.0 * 1.0 - 1;
        self->r3_real_val = self->r3_val / 1000.0 * 1.0 - 1;
        self->r4_real_val = self->r4_val / 1000.0 * 1.0 - 1;
        self->r5_real_val = self->r5_val / 1000.0 * 1.0 - 1;
        self->r6_real_val = self->r6_val / 1000.0 * 1.0 - 1;
        self->r7_real_val = self->r7_val / 1000.0 * 1.0 - 1;
        self->r8_real_val = self->r8_val / 1000.0 * 1.0 - 1;
        self->r9_real_val = self->r9_val / 1000.0 * 1.0 - 1;
        self->t1_real_val = self->t1_val / 1000.0 * 1.0 - 1;
        self->t2_real_val = self->t2_val / 1000.0 * 1.0 - 1;
        self->t3_real_val = self->t3_val / 1000.0 * 1.0 - 1;
        std::cout<<"R:"<<std::endl<<self->r1_real_val <<" "<<self->r2_real_val<<" "<<self->r3_real_val<<std::endl<<
        self->r4_real_val<<" "<<self->r5_real_val<<" "<<self->r6_real_val<<std::endl<<
        self->r7_real_val<<" "<<self->r8_real_val<<" "<<self->r9_real_val<<std::endl<<"T:"<<std::endl<<
        self->t1_real_val<<" "<<self->t2_real_val<<" "<<self->t3_real_val<<std::endl<<"-------------------"<<std::endl;
        self->nh_.setParam("/intrinsic_bar_val/r1_val",self->r1_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r2_val",self->r2_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r3_val",self->r3_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r4_val",self->r4_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r5_val",self->r5_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r6_val",self->r6_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r7_val",self->r7_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r8_val",self->r8_real_val);
        self->nh_.setParam("/intrinsic_bar_val/r9_val",self->r9_real_val);
        self->nh_.setParam("/intrinsic_bar_val/t1_val",self->t1_real_val);
        self->nh_.setParam("/intrinsic_bar_val/t2_val",self->t2_real_val);
        self->nh_.setParam("/intrinsic_bar_val/t3_val",self->t3_real_val);
    }

    inline void saveYamlFile(std::string Filename)
    {
        FileStorage fs(Filename, FileStorage::WRITE);
        Mat M = Mat::zeros(4,4,CV_64FC1);
        M.at<float>(0, 0) = r1_real_val;
        M.at<float>(0, 1) = r2_real_val;
        M.at<float>(0, 2) = r3_real_val;
        M.at<float>(0, 3) = t1_real_val;
        M.at<float>(1, 0) = r4_real_val;
        M.at<float>(1, 1) = r5_real_val;
        M.at<float>(1, 2) = r6_real_val;
        M.at<float>(1, 3) = t2_real_val;
        M.at<float>(2, 0) = r7_real_val;
        M.at<float>(2, 1) = r8_real_val;
        M.at<float>(2, 2) = r9_real_val;
        M.at<float>(2, 3) = t3_real_val;
        M.at<float>(3, 3) = 1;
        if(fs.isOpened())
        {
            fs << "INTRINSIC" << M;
            fs.release();
        }
        else
        {
            ROS_INFO("YAML文件创建失败!\n");
        }
    }
};