
//rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw
#include "utility.h"



class ImageProcess{
private:
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Subscriber subImage_raw;
    image_transport::Publisher pubImage_reprojection;

    std_msgs::Header ImageHeader;

    bool newImageraw;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_pub;
public:
    ImageProcess():
    it(nh){
        subImage_raw = it.subscribe("/usb_cam/image_raw",1,&ImageProcess::image_processhandler,this);
        
        pubImage_reprojection = it.advertise("/usb_cam/image_reprojection",1);
        allocateMem();
        paraInit();
    }

    void allocateMem(){
        cv_ptr.reset(new cv_bridge::CvImage);
    }
    void paraInit(){
        newImageraw = false;
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
    void rgb2grey(){
        //cv::cvtColor(cv_ptr->image, img_pub, CV_RGB2GRAY);  //转换成灰度图象
        img_pub = cv_ptr->image;
        if(!img_pub.empty()){

            cv::waitKey(5);
        }
    }
    void pubImage(){
        sensor_msgs::ImagePtr Imagemsg = cv_bridge::CvImage(ImageHeader, "bgr8", img_pub).toImageMsg();
        pubImage_reprojection.publish(Imagemsg);
    }

    void run(){
        if(newImageraw){
            newImageraw = false;
            rgb2grey();
            pubImage();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m visualizatoin Started.");
    ImageProcess IP;
    //ros::spin();
    ros::Rate rate(2);
    while(ros::ok()){
        ros::spinOnce();
        IP.run();
        rate.sleep();
    }
    return 0;
}