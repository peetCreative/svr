#include "SVR.h"
#include "ROSSVR.h"

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <iostream>

PLUGINLIB_EXPORT_CLASS( ROSSVRNodelet, nodelet::Nodelet )

ROSSVRNodelet::ROSSVRNodelet()
:
    graphicsGameState(nullptr),
    graphicsSystem(nullptr)
{
}

void ROSSVRNodelet::onInit()
{
    NODELET_DEBUG("Initializing ROSSVRNodelet...");
    ros::NodeHandle& nh = getNodeHandle();

    mSubImageLeft = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, "left/image_rect", 20);
    mSubImageRight = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, "right/image_rect", 20);
    mSubCamInfoLeft = nh.subscribe(
        "left/camera_info", 1, &ROSSVRNodelet::newCameraInfoCallbackLeft, this);
    mSubCamInfoRight = nh.subscribe(
        "right/camera_info", 1, &ROSSVRNodelet::newCameraInfoCallbackRight, this);

    mApproximateSync.reset(
        new ApproximateSync(ApproximatePolicy(20),
        *mSubImageLeft, *mSubImageRight));
    mApproximateSync->registerCallback(
        boost::bind( &ROSSVRNodelet::newImageCallback, this, _1, _2));

//     ros::NodeHandle& nhPriv = getPrivateNodeHandle();
    createSystems( &graphicsGameState, &graphicsSystem, false );

    try
    {
        graphicsSystem->initialize( getWindowTitle() );

        if( graphicsSystem->getQuit() )
        {
            graphicsSystem->deinitialize();

            destroySystems( graphicsGameState, graphicsSystem);

            return; //User cancelled config
        }

        mRenderWindow = graphicsSystem->getRenderWindow();

        graphicsSystem->createScene01();

    #if OGRE_USE_SDL2
        //Do this after creating the scene for easier the debugging (the mouse doesn't hide itself)
        SdlInputHandler *mInputHandler = graphicsSystem->getInputHandler();
        mInputHandler->setGrabMousePointer( true );
        mInputHandler->setMouseVisible( false );
        mInputHandler->setMouseRelative( true );
    #endif
        mTimer = Ogre::Timer();
        mStartTime = mTimer.getMicroseconds();
        mTimeSinceLast = 0;
    }
    catch( Ogre::Exception &e )
    {
    //TODO: Do sth more intelligent
        destroySystems( graphicsGameState, graphicsSystem );
        throw e;
    }
    catch( ... )
    {
        destroySystems( graphicsGameState, graphicsSystem );
    }

}

void ROSSVRNodelet::newImageCallback(
    const sensor_msgs::Image::ConstPtr& imgLeft,
    const sensor_msgs::Image::ConstPtr& imgRight)
{
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
        graphicsSystem->beginFrameParallel();
        graphicsSystem->getOvrCompositorListener()->setImgPtr(
            &(cv_ptr_left->image), &(cv_ptr_right->image));
        graphicsSystem->update( static_cast<float>( mTimeSinceLast ) );
        graphicsSystem->finishFrameParallel();

        //TODO: must maybe be calculated before?
        Ogre::uint64 endTime = mTimer.getMicroseconds();
        mTimeSinceLast = (endTime - mStartTime) / 1000000.0;
        mTimeSinceLast = std::min( 1.0, timeSinceLast ); //Prevent from going haywire.
        accumulator += mTimeSinceLast;
        mStartTime = endTime;
        if(graphicsSystem->getQuit())
        {
            graphicsSystem->destroyScene();
            graphicsSystem->deinitialize();
            destroySystems( graphicsGameState, graphicsSystem);
        }
    }
    catch( Ogre::Exception &e )
    {
        destroySystems( graphicsGameState, graphicsSystem );
        throw e;
    }
    catch( ... )
    {
        destroySystems( graphicsGameState, graphicsSystem );
    }
}

void ROSSVRNodelet::newCameraInfoCallbackLeft(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo )
{
    newCameraInfoCallback(camInfo, LEFT);
}
void ROSSVRNodelet::newCameraInfoCallbackRight(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo )
{
    newCameraInfoCallback(camInfo, RIGHT);
}

void ROSSVRNodelet::newCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo,
    int leftOrRight)
{
    graphicsSystem->getOvrCompositorListener()->setCameraConfig(
        camInfo->width,
        camInfo->height,
        camInfo->K[0], camInfo->K[4],
        camInfo->K[2], camInfo->K[5],
        leftOrRight
    );
}
