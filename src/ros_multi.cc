/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageLeftSide(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRightSide(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    std::atomic<bool> shutdown_requested{false};

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf;
    queue<sensor_msgs::ImageConstPtr> imgRightBuf;
    queue<sensor_msgs::ImageConstPtr> imgLeftSideBuf;
    queue<sensor_msgs::ImageConstPtr> imgRightSideBuf;

    std::mutex mBufMutexLeft, mBufMutexRight, mBufMutexLeftSide, mBufMutexRightSide;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Multi");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");

    sensor_type = ORB_SLAM3::System::MULTI;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin, 0, "");

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);
    
    ros::Subscriber sub_imu = node_handler.subscribe("/alphasense/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img_left = node_handler.subscribe("/alphasense/cam1/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = node_handler.subscribe("/alphasense/cam0/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);
    ros::Subscriber sub_img_leftside = node_handler.subscribe("/alphasense/cam4/image_raw", 100, &ImageGrabber::GrabImageLeftSide, &igb);
    ros::Subscriber sub_img_rightside = node_handler.subscribe("/alphasense/cam3/image_raw", 100, &ImageGrabber::GrabImageRightSide, &igb);
    
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    igb.shutdown_requested = true;
    sync_thread.join(); 

    pSLAM->Shutdown();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    try {
        pSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
        pSLAM->SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
        pSLAM->SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception when saving trajectory: " << e.what());
    }

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ROS_INFO("Received image from /camera/left/image_raw");

    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ROS_INFO("Received image from /camera/right/image_raw");

    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

void ImageGrabber::GrabImageLeftSide(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ROS_INFO("Received image from /camera/leftside/image_raw");

    mBufMutexLeftSide.lock();
    if (!imgLeftSideBuf.empty())
        imgLeftSideBuf.pop();
    imgLeftSideBuf.push(img_msg);
    mBufMutexLeftSide.unlock();
}

void ImageGrabber::GrabImageRightSide(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ROS_INFO("Received image from /camera/rightside/image_raw");

    mBufMutexRightSide.lock();
    if (!imgRightSideBuf.empty())
        imgRightSideBuf.pop();
    imgRightSideBuf.push(img_msg);
    mBufMutexRightSide.unlock();
}

cv::Mat EnhanceDarkImage(const cv::Mat& input_gray)
{
    cv::Scalar meanVal = cv::mean(input_gray);
    double mean_brightness = meanVal[0];

    double gamma_val;
    double clip_limit;
    cv::Size tile_grid_size;

    if (mean_brightness < 50)
    {
        gamma_val = 0.4;
        clip_limit = 8.0;
        tile_grid_size = cv::Size(6, 6);  
    }
    else if (mean_brightness < 60)
    {
        gamma_val = 0.6;
        clip_limit = 4.0;
        tile_grid_size = cv::Size(8, 8);
    }
    else if (mean_brightness < 100)
    {
        gamma_val = 0.8;
        clip_limit = 2.0;
        tile_grid_size = cv::Size(8, 8);
    }
    else
    {
        gamma_val = 1.0;  
        clip_limit = 1.0; 
        tile_grid_size = cv::Size(16, 16); 
    }

    cv::Mat img_float, img_gamma;
    input_gray.convertTo(img_float, CV_32F, 1.0 / 255.0);
    cv::pow(img_float, gamma_val, img_float);
    img_float.convertTo(img_gamma, CV_8U, 255.0);

    cv::Mat img_equalized;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(clip_limit);
    clahe->setTilesGridSize(tile_grid_size);
    clahe->apply(img_gamma, img_equalized);

    cv::Mat contrast_final;
    cv::normalize(img_equalized, contrast_final, 0, 255, cv::NORM_MINMAX);

    return contrast_final;
}


cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    if (cv_ptr->image.empty())
    {
        ROS_WARN("Empty image received.");
        return cv::Mat();
    }

    cv::Mat img_equalized = EnhanceDarkImage(cv_ptr->image);
    return img_equalized;
    // return cv_ptr->image;
}


void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(ros::ok() && !shutdown_requested)
    {

        cv::Mat imLeft, imRight, imLeftSide, imRightSide;
        double tImLeft = 0, tImRight = 0, tImLeftSide = 0, tImRightSide = 0;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!imgLeftSideBuf.empty()&&!imgRightSideBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            // std::cout << ">>> entered sync block" << std::endl;

            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();
            tImLeftSide = imgLeftSideBuf.front()->header.stamp.toSec();
            tImRightSide = imgRightSideBuf.front()->header.stamp.toSec();

            // std::cout << "tImLeft=" << tImLeft
            //           << " tImRight=" << tImRight
            //           << " tImLeftSide=" << tImLeftSide
            //           << " tImRightSide=" << tImRightSide << std::endl;

            // Right
            this->mBufMutexRight.lock();
            while (fabs(tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            // LeftSide
            this->mBufMutexLeftSide.lock();
            while (fabs(tImLeft - tImLeftSide) > maxTimeDiff && imgLeftSideBuf.size() > 1)
            {
                imgLeftSideBuf.pop();
                tImLeftSide = imgLeftSideBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeftSide.unlock();

            // RightSide
            this->mBufMutexRightSide.lock();
            while (fabs(tImLeft - tImRightSide) > maxTimeDiff && imgRightSideBuf.size() > 1)
            {
                imgRightSideBuf.pop();
                tImRightSide = imgRightSideBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRightSide.unlock();

            if (fabs(tImLeft - tImRight) > maxTimeDiff || fabs(tImLeft - tImLeftSide) > maxTimeDiff || fabs(tImLeft - tImRightSide) > maxTimeDiff)
            {
                // std::cout << ">>> big time difference, skip" 
                //           << " L-R=" << fabs(tImLeft - tImRight)
                //           << " L-LS=" << fabs(tImLeft - tImLeftSide)
                //           << " L-RS=" << fabs(tImLeft - tImRightSide) << std::endl;
                continue;
            }

            if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
            {
                // std::cout << ">>> waiting for IMU, tImLeft=" << tImLeft 
                //           << " imuBuf.back=" << mpImuGb->imuBuf.back()->header.stamp.toSec() << std::endl;
                continue;
            }

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            ros::Time msg_time = imgLeftBuf.front()->header.stamp;
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            this->mBufMutexLeftSide.lock();
            imLeftSide = GetImage(imgLeftSideBuf.front());
            imgLeftSideBuf.pop();
            this->mBufMutexLeftSide.unlock();

            this->mBufMutexRightSide.lock();
            imRightSide = GetImage(imgRightSideBuf.front());
            imgRightSideBuf.pop();
            this->mBufMutexRightSide.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // std::cout << ">>> calling TrackMC, vImuMeas.size=" << vImuMeas.size() << std::endl;

            Sophus::SE3f Tcw = pSLAM->TrackMC(imLeft, imRight, imLeftSide, imRightSide, tImLeft, vImuMeas);

            // std::cout << ">>> TrackMC done" << std::endl;

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}