#include "ROSSVR.h"
#include "SVR.h"
#include "SVRGraphicsSystem.h"

#include "OgreCommon/SdlInputHandler.h"
#include "OgreWindow.h"
#include "OgreTimer.h"

#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_calibration_parsers/parse_yml.h>

#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <iostream>
#include <thread>
#include <memory>


using namespace Demo;

ROSSVR::ROSSVR()
:
    mSVR(nullptr),
    mIsCameraInfoInit{false, false}
{
}

void ROSSVR::onInit()
{
    NODELET_DEBUG("Initializing ROSSVR...");
    ros::NodeHandle& nh = getNodeHandle();

}

int ROSSVR::init()
{
    mSubImageLeft = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, "/stereo/resized/left/image_raw", 20);
    mSubImageRight = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, "/stereo/resized/right/image_raw", 20);
    mSubCamInfoLeft = nh.subscribe(
        "/stereo/resized/left/camera_info", 1, &ROSSVR::newCameraInfoCallbackLeft, this);
    mSubCamInfoRight = nh.subscribe(
        "/stereo/resized/right/camera_info", 1, &ROSSVR::newCameraInfoCallbackRight, this);

    mApproximateSync.reset(
        new ApproximateSync(ApproximatePolicy(20),
        *mSubImageLeft, *mSubImageRight));
    mApproximateSync->registerCallback(
        boost::bind( &ROSSVR::newImageCallback, this, _1, _2));

//     ros::NodeHandle svr_nh(node, "svr");
    mSVR.reset(new SVR(false));
    mSVR->init(CONST_MAT);

    // spawn device thread
    running_ = true;
    mSVRThread = 
        std::shared_ptr< std::thread > (
            new std::thread(boost::bind(&ROSSVR::mainloop, this)));
    std::cout << "end of init" << std::endl;
}

void ROSSVR::mainloop()
{
    std::cout << "startloop" << std::endl;
    while (running_ && !mSVR->getQuit())
    {
        std::cout << "spin" << std::endl;
        mSVR->spin();
    }
    //TODO:destroy system
    //delete mSVR;
}

void ROSSVR::newImageCallback(
    sensor_msgs::Image::ConstPtr& imgLeft,
    sensor_msgs::Image::ConstPtr& imgRight)
{
    std::cout << "new image" << std::endl;
    if (!mIsCameraInfoInit[LEFT] || !mIsCameraInfoInit[RIGHT])
        return;
    cv_bridge::CvImageConstPtr cv_ptr_left;
    cv_bridge::CvImageConstPtr cv_ptr_right;
    try
    {
        cv_ptr_left = cv_bridge::toCvShare(
            imgLeft, sensor_msgs::image_encodings::BGR8 );
        cv_ptr_right = cv_bridge::toCvShare(
            imgRight, sensor_msgs::image_encodings::BGR8 );
    }
    catch (cv_bridge::Exception& e)
    {
        NODELET_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try
    {
        mSVR->getGraphicsSystem()->getOvrCompositorListener()->setImgPtr(
            &(cv_ptr_left->image), &(cv_ptr_right->image));
    }
    catch( Ogre::Exception &e )
    {
        std::cout << "ROS oh sth went wront with OGRE!!" << std::endl;
//         delete mSVR;
        //TODO: let's unregister this as well
        throw e;
    }
    catch( ... )
    {
//         destroySystems( graphicsGameState, graphicsSystem );
    }
}

void ROSSVR::newCameraInfoCallbackLeft(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo )
{
    if (!mIsCameraInfoInit[LEFT])
    {
        std::cout << "camera_info_left" << std::endl;
        newCameraInfoCallback(camInfo, LEFT);
        mIsCameraInfoInit[LEFT] = true;
        //TODO: maybe unregister
    }
}
void ROSSVR::newCameraInfoCallbackRight(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo )
{
    if (!mIsCameraInfoInit[RIGHT])
    {
        std::cout << "camera_info_right" << std::endl;
        newCameraInfoCallback(camInfo, RIGHT);
        mIsCameraInfoInit[RIGHT] = true;
        //TODO: maybe unregister
    }
}

void ROSSVR::newCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo,
    int leftOrRight)
{
    mSVR->getGraphicsSystem()->getOvrCompositorListener()->setCameraConfig(
        camInfo->width,
        camInfo->height,
        camInfo->K[0], camInfo->K[4],
        camInfo->K[2], camInfo->K[5],
        leftOrRight
    );
}

PLUGINLIB_EXPORT_CLASS( ROSSVR, nodelet::Nodelet )
