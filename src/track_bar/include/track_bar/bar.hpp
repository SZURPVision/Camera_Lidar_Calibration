#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/core.hpp"
#include "ros/ros.h"
#include "Eigen/Dense"
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
        // this->r1_val = 0;
        // this->r2_val = 0;
        // this->r3_val = 0;
        // this->r4_val = 0;
        // this->r5_val = 0;
        // this->r6_val = 0;
        // this->r7_val = 0;
        // this->r8_val = 0;
        // this->r9_val = 0;
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
        // std::cout<<"yes";
        createTrackbar("roll","bar",&roll,3600,&bar::on_callback,this);
        createTrackbar("pitch","bar",&pitch,3600,&bar::on_callback,this);
        createTrackbar("yaw","bar",&yaw,3600,&bar::on_callback,this);
        // createTrackbar("r4_val","bar",&r4_val,2000,&bar::on_callback,this);
        // createTrackbar("r5_val","bar",&r5_val,2000,&bar::on_callback,this);
        // createTrackbar("r6_val","bar",&r6_val,2000,&bar::on_callback,this);
        // createTrackbar("r7_val","bar",&r7_val,2000,&bar::on_callback,this);
        // createTrackbar("r8_val","bar",&r8_val,2000,&bar::on_callback,this);
        // createTrackbar("r9_val","bar",&r9_val,2000,&bar::on_callback,this);
        createTrackbar("t1_val","bar",&t1_val,2000,&bar::on_callback,this);
        createTrackbar("t2_val","bar",&t2_val,2000,&bar::on_callback,this);
        createTrackbar("t3_val","bar",&t3_val,2000,&bar::on_callback,this);
    }
private:
    // int r1_val, r2_val, r3_val, r4_val, r5_val, r6_val, r7_val, r8_val,r9_val;
    int t1_val, t2_val, t3_val;
    double r1_real_val, r2_real_val, r3_real_val, r4_real_val, r5_real_val, r6_real_val, r7_real_val, r8_real_val, r9_real_val;
    double t1_real_val, t2_real_val, t3_real_val;
    int roll,pitch,yaw;
    double roll_real,pitch_real,yaw_real;
    ros::NodeHandle nh_;
    static void on_callback(int value, void* userdata)
    {
        // 旋转顺序xyz -> roll pitch yaw
        bar* self = static_cast<bar*>(userdata);
        // self->on_callback(value);
        self->roll_real = (self->roll-1800)/1800.0 * M_PI;
        self->pitch_real = (self->pitch-1800)/1800.0 * M_PI;
        self->yaw_real = (self->yaw-1800)/1080.0 * M_PI;
        // self->r1_real_val = cos(self->pitch_real)*cos(self->yaw_real); 
        // self->r2_real_val = sin(self->roll_real)*sin(self->pitch_real)*cos(self->yaw_real)-cos(self->roll_real)*sin(self->pitch_real);
        // self->r3_real_val = cos(self->roll_real)*sin(self->pitch_real)*cos(self->yaw_real)+sin(self->roll_real)*sin(self->yaw_real);
        // self->r4_real_val = cos(self->pitch_real)*sin(self->yaw_real);
        // self->r5_real_val = sin(self->roll_real)*sin(self->pitch_real)*sin(self->yaw_real)+cos(self->roll_real)*cos(self->yaw_real);
        // self->r6_real_val = cos(self->roll_real)*sin(self->pitch_real)*sin(self->roll_real)-sin(self->roll_real)*cos(self->pitch_real);
        // self->r7_real_val = -sin(self->pitch_real);
        // self->r8_real_val = sin(self->roll_real)*cos(self->pitch_real);
        // self->r9_real_val = cos(self->roll_real)*cos(self->pitch_real);
        Eigen::Vector3d euler_angle(self->roll_real, self->pitch_real, self->yaw_real);

        // 使用Eigen库将欧拉角转换为旋转矩阵
        Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
        rotation_matrix1 = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());
        std::cout << "\nrotation matrix1 =\n" << rotation_matrix1 << std::endl; //<< endl;
        self->r1_real_val = rotation_matrix1(0,0); 
        self->r2_real_val = rotation_matrix1(0,1);
        self->r3_real_val = rotation_matrix1(0,2);
        self->r4_real_val = rotation_matrix1(1,0);
        self->r5_real_val = rotation_matrix1(1,1);       
        self->r6_real_val = rotation_matrix1(1,2);
        self->r7_real_val = rotation_matrix1(2,0);
        self->r8_real_val = rotation_matrix1(2,1);
        self->r9_real_val = rotation_matrix1(2,2);
        // self->r1_real_val = self->r1_val / 1000.0 * 1.0 - 1;
        // self->r2_real_val = self->r2_val / 1000.0 * 1.0 - 1;
        // self->r3_real_val = self->r3_val / 1000.0 * 1.0 - 1;
        // self->r4_real_val = self->r4_val / 1000.0 * 1.0 - 1;
        // self->r5_real_val = self->r5_val / 1000.0 * 1.0 - 1;
        // self->r6_real_val = self->r6_val / 1000.0 * 1.0 - 1;
        // self->r7_real_val = self->r7_val / 1000.0 * 1.0 - 1;
        // self->r8_real_val = self->r8_val / 1000.0 * 1.0 - 1;
        // self->r9_real_val = self->r9_val / 1000.0 * 1.0 - 1;
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