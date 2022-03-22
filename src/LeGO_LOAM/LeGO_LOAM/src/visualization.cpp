
//rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw
#include "utility.h"



class ImageProcess{
private:
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Subscriber subImage_raw;
    image_transport::Publisher pubImage_reprojection;
    ros::Subscriber subLidar;
    std_msgs::Header ImageHeader;

    bool newImageraw;
    bool newLaserCloudCornerLast;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_pub;
    pcl::PointCloud<PointType>::Ptr LaserCloudPoints;
    Eigen::Matrix3f Camera_I;
    Eigen::Matrix3f Camera_E_R;
    Eigen::Vector3f Camera_E_T;

    double timeLaserCloudCornerLast;
public:
    ImageProcess():
    it(nh){
        subImage_raw = it.subscribe("/usb_cam/image_raw",1,&ImageProcess::image_processhandler,this);
        subLidar = nh.subscribe<sensor_msgs::PointCloud2>("/Cloudfortest",2,&ImageProcess::PointCloudhander,this);
        pubImage_reprojection = it.advertise("/usb_cam/image_reprojection",1);
        allocateMem();
        paraInit();
    }

    void allocateMem(){
        LaserCloudPoints.reset(new pcl::PointCloud<PointType>);
        cv_ptr.reset(new cv_bridge::CvImage);
    }
    void paraInit(){
        Camera_E_T << 0.1403 , -0.0075 , -0.0969;
        Camera_E_R << -0.0315 , -1.0210 , -0.0212 , 0.1093 , -0.0120 , -0.9862 , 0.9754 , -0.0515 , 0.1088;
        Camera_I << 484.009289 , 0.000000 , 298.361019 , 0.000000 , 481.594124 , 251.230678 , 0.000000 , 0.000000 , 1.000000;
        newImageraw = false;
        newLaserCloudCornerLast = false;
    }
    void clearCloud(){
        LaserCloudPoints->clear();
    }
    void image_processhandler(const sensor_msgs::ImageConstPtr& msg){
        ImageHeader = msg->header;
        try
        {
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
        catch(cv_bridge::Exception& e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        newImageraw = true;
        //run();
    }
    void PorjectPointCloud(){
        //cv::cvtColor(cv_ptr->image, img_pub, CV_RGB2GRAY);  //转换成灰度图象
        for(auto point : *LaserCloudPoints){
            Eigen::Vector3f p;
            
            p << point.x , point.y , point.z;
            p = Camera_I * Camera_E_R * (p - Camera_E_T);

            cv::Point uv;
            uv.x = p(0) / p(2);
            uv.y = p(1) / p(2);
            cv::circle(cv_ptr->image,uv,1,cv::Scalar(0, 0, 255));
        }
    }
    void pubImage(){
        sensor_msgs::ImagePtr Imagemsg = cv_bridge::CvImage(ImageHeader, "bgr8", cv_ptr->image).toImageMsg();
        pubImage_reprojection.publish(Imagemsg);
    }
    void PointCloudhander(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudCornerLast = msg->header.stamp.toSec();
        LaserCloudPoints->clear();
        pcl::fromROSMsg(*msg, *LaserCloudPoints);
        newLaserCloudCornerLast = true;
    }

    void run(){
        if(newImageraw && newLaserCloudCornerLast){
            static int times = 0;
            newImageraw = false;
            newLaserCloudCornerLast = false;
            cout << "runing" << times++ << endl;
            PorjectPointCloud();
            pubImage();
            clearCloud();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m visualizatoin Started.");
    ImageProcess IP;
    //ros::spin();
    while(ros::ok()){
        ros::spinOnce();
        IP.run();
    }
    return 0;
}