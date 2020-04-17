#include "SVR.h"

#include "OgrePrerequisites.h"
#include <iostream>

#include "OgreCommon/SdlInputHandler.h"

#include "OgreWindow.h"
#include "OgreTimer.h"
#include "OgreLogManager.h"

#include "Threading/OgreThreads.h"

#include "SVRGraphicsSystem.h"
#include "SVRGameState.h"

#include "opencv2/opencv.hpp"
#include <experimental/filesystem>
#include <libconfig.h++>

#ifdef USE_ROS
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#endif

using namespace cv;
namespace fs = std::experimental::filesystem;


#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <pwd.h>
    #include <errno.h>
#endif

namespace Demo
{
    SVR::SVR(bool showConfigDialog,
        InputType inputType
    ) :
        mInputType(inputType),
        mImageSeriesInput(nullptr),
        mImageTimestampInput(nullptr),
        mQuit(false),
        mIsCameraInfoInit{false, false}
    {
        mGameState = new SVRGameState( "SVR" );

        mGraphicsSystem =
            new SVRGraphicsSystem( mGameState, showConfigDialog );

        mGameState->_notifyGraphicsSystem( mGraphicsSystem );

        mTimer = Ogre::Timer();
        mStartTime = mTimer.getMicroseconds();

        mAccumulator = 1.0 / 60.0;
        mTimeSinceLast = 1.0 / 60.0;

        switch(mInputType)
        {
            case VIDEO:
                mVideoInput = new VideoInput();
                initVideoInput();
                break;
            case IMG_SERIES:
                mImageSeriesInput = new ImageSeriesInput();
                initImgSeries();
                break;
            case IMG_TIMESTAMP:
                mImageTimestampInput = new ImageTimestampInput();
                initImgTimestamp();
                break;
            default: break;
        }
    }

    SVR::~SVR()
    {
        mGraphicsSystem->deinitialize();
        delete mGraphicsSystem;
        delete mGameState;

        switch(mInputType)
        {
            case VIDEO:
                delete mVideoInput;
                break;
            case IMG_SERIES:
                delete mImageSeriesInput;
                break;
            case IMG_TIMESTAMP:
                delete mImageTimestampInput;
                break;
            default: break;
        }
    }

    bool SVR::init()
    {
        try
        {
            mGraphicsSystem->initialize( getWindowTitle() );

            if( mGraphicsSystem->getQuit() )
            {
                mGraphicsSystem->deinitialize();

                Ogre::LogManager::getSingleton().logMessage(
                    "graphics System get quit");
                return false; //User cancelled config
            }

            mRenderWindow = mGraphicsSystem->getRenderWindow();

            mGraphicsSystem->createScene01();


        #if OGRE_USE_SDL2
            //Do this after creating the scene for easier the debugging (the mouse doesn't hide itself)
            SdlInputHandler *inputHandler = mGraphicsSystem->getInputHandler();
            inputHandler->setGrabMousePointer( true );
            inputHandler->setMouseVisible( false );
            inputHandler->setMouseRelative( true );
        #endif
        }
        catch( Ogre::Exception &e )
        {
            Ogre::LogManager::getSingleton().logMessage("something .. oh see Ogre is brocken. AS ALWAYS..");
            throw e;
            return false;
        }
        catch( ... )
        {
            Ogre::LogManager::getSingleton().logMessage("something relly bad happend");
            return false;
        }
        return true;
    }

    void SVR::spin()
    {
        mGraphicsSystem->beginFrameParallel();
        mGraphicsSystem->update( static_cast<float>( mTimeSinceLast ) );
        mGraphicsSystem->finishFrameParallel();

        if( !mRenderWindow->isVisible() )
        {
            //Don't burn CPU cycles unnecessary when we're minimized.
            Ogre::Threads::Sleep( 500 );
        }

        Ogre::uint64 endTime = mTimer.getMicroseconds();
        mTimeSinceLast = (endTime - mStartTime) / 1000000.0;
        mTimeSinceLast = std::min( 1.0, mTimeSinceLast ); //Prevent from going haywire.
        mAccumulator += mTimeSinceLast;
        mStartTime = endTime;
    }

    const char* SVR::getWindowTitle(void)
    {
        return "SVR";
    }

    bool SVR::getQuit()
    {
        return  mQuit || mGraphicsSystem && mGraphicsSystem->getQuit();
    }

    void SVR::setQuit()
    {
        mQuit = true;
    }

    bool SVR::initVideoInput()
    {
        mVideoInput->capture.set(CV_CAP_PROP_MODE,  CV_CAP_MODE_RGB );
        mVideoInput->capture = VideoCapture("/home/peetcreative/SurgicalData/ForPeter/Video.avi");
        if (!mVideoInput->capture.isOpened()) {
            Ogre::LogManager::getSingleton().logMessage( "Video could not be opened" );
            return false;
        }
        // Default resolution of the frame is obtained.The default resolution is system dependent.
        mVideoInput->captureFrameWidth =
            mVideoInput->capture.get(CV_CAP_PROP_FRAME_WIDTH);
        mVideoInput->captureFrameHeight =
            mVideoInput->capture.get(CV_CAP_PROP_FRAME_HEIGHT);
        mVideoInput->captureFramePixelFormat =
            mVideoInput->capture.get(CV_CAP_PROP_FORMAT);
        return true;
    }

    void SVR::updateVideoInput()
    {
        cv::Mat mMat;

        // Capture frame-by-frame
        mVideoInput->capture >> mMat; //1920/1080
        if(mMat.empty())
        {
            Ogre::LogManager::getSingleton().logMessage("mMat empty");
            return;
        }
        cv::Rect lrect(0,540, 1920, 540);
        cv::Rect rrect(0,0, 1920, 540);
        cv::Mat imageOrigLeft = mMat(lrect);
        cv::Mat imageOrigRight = mMat(rrect);
        if( imageOrigLeft.empty() || imageOrigRight.empty() )
        {
            return;
        }
        mGraphicsSystem->getOvrCompositorListener()->setImgPtr( &imageOrigLeft, &imageOrigRight );
    }

    bool SVR::initImgSeries()
    {
        mImageSeriesInput->imgPath = "/tmp/rect/left_undist_rect01";
        return true;
    }

    void SVR::updateImgSeries()
    {
        if (mImageSeriesInput->imgCnt >= 1000)
            mImageSeriesInput->imgCnt = 0;
        //TODO: change to left and right name
        std::stringstream ssl;
        ssl << mImageSeriesInput->imgPath
            << std::setfill('0') << std::setw(3)
            << mImageSeriesInput->imgCnt
            << ".png";
        std::stringstream ssr;
        ssr << mImageSeriesInput->imgPath
            << std::setfill('0') << std::setw(3)
            << mImageSeriesInput->imgCnt
            << ".png";
        Ogre::LogManager::getSingleton().logMessage( "fill texture with " + ssl.str() );
        Ogre::LogManager::getSingleton().logMessage( "fill texture with " + ssr.str() );
        mImageSeriesInput->imgCnt++;
        cv::Mat imageOrigLeft = imread(ssl.str());
        cv::Mat imageOrigRight = imread(ssr.str());
        mGraphicsSystem->getOvrCompositorListener()->setImgPtr( &imageOrigLeft, &imageOrigRight );
    }

    bool SVR::initImgTimestamp()
    {
        mFileIteratorLeft = fs::directory_iterator("/tmp/new_images");
        return true;
    }

    void SVR::updateImgTimestamp()
    {
        fs::directory_entry entry_left = *mFileIteratorLeft++;
        std::string path_str_left = entry_left.path().string();
        size_t pos_left = path_str_left.rfind("left");
        std::string path_str_right = path_str_left.substr(0, pos_left) +
            "right" +
            path_str_left.substr(pos_left + 4, path_str_left.length());
        fs::path path_right(path_str_right);
        if(!exists(path_right))
            return;
        cv::Mat imageOrigLeft = imread(path_str_right);
        cv::Mat imageOrigRight = imread(path_str_left);
        mGraphicsSystem->getOvrCompositorListener()->setImgPtr( &imageOrigLeft, &imageOrigRight );
    }

#ifdef USE_ROS
    void SVR::subscribeROSTopics(ros::NodeHandle mNh)
    {
        mSubImageLeft = new
            message_filters::Subscriber<sensor_msgs::Image> (
                mNh, "/stereo/left/image_undist_rect", 20);
        mSubImageRight = new 
            message_filters::Subscriber<sensor_msgs::Image> (
                mNh, "/stereo/right/image_undist_rect", 20);
        mSubCamInfoLeft = mNh.subscribe(
            "/stereo/left/camera_info", 1,
            &SVR::newROSCameraInfoCallbackLeft, this);
        mSubCamInfoRight = mNh.subscribe(
            "/stereo/right/camera_info", 1,
            &SVR::newROSCameraInfoCallbackRight, this);
        mApproximateSync.reset(
            new ApproximateSync(
                ApproximatePolicy(20),
                *mSubImageLeft, *mSubImageRight));
        mApproximateSync->registerCallback(
            boost::bind( &SVR::newROSImageCallback, this,_1, _2));
    }

    void SVR::newROSImageCallback(
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight
        )
    {
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
            std::cout <<"cv_bridge exception: " << e.what() << std::endl;
            return;
        }
        try
        {
            mGraphicsSystem->getOvrCompositorListener()->setImgPtr(
                &(cv_ptr_left->image), &(cv_ptr_right->image));
        }
        catch( Ogre::Exception &e )
        {
            std::cout << "ROS oh sth went wront with OGRE!!" << std::endl;
            //TODO: let's unregister this as well
            throw e;
        }
        catch( ... )
        {
    //         destroySystems( graphicsGameState, graphicsSystem );
        }
    }

    void SVR::newROSCameraInfoCallbackLeft(
        const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        std::cout << "camera_info_left" << std::endl;
        if (!mIsCameraInfoInit[LEFT])
        {
            std::cout << "camera_info_left" << std::endl;
            newROSCameraInfoCallback(camInfo, LEFT);
            mIsCameraInfoInit[LEFT] = true;
            mSubCamInfoLeft.shutdown();
        }
    }
    void SVR::newROSCameraInfoCallbackRight(
        const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        std::cout << "camera_info_right" << std::endl;
        if (!mIsCameraInfoInit[RIGHT])
        {
            std::cout << "camera_info_right" << std::endl;
            newROSCameraInfoCallback(camInfo, RIGHT);
            mIsCameraInfoInit[RIGHT] = true;
            mSubCamInfoRight.shutdown();
        }
    }

    void SVR::newROSCameraInfoCallback(
        const sensor_msgs::CameraInfo::ConstPtr& camInfo,
        int leftOrRight)
    {
        mGraphicsSystem->getOvrCompositorListener()->setCameraConfig(
            camInfo->width,
            camInfo->height,
            camInfo->K[0], camInfo->K[4],
            camInfo->K[2], camInfo->K[5],
            leftOrRight
        );
    }
#endif
}


using namespace Demo;
int main( int argc, char *argv[] )
{
    bool config_dialog = false;
#ifdef USE_ROS
    InputType input = ROS;
#else
    InputType input = VIDEO;
#endif
    for (int i = 1; i < argc; i++)
    {
        config_dialog = config_dialog || std::strcmp(argv[i], "--config-dialog") == 0;
        if (std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc)
        {
            if (std::strcmp(argv[i+1], "VIDEO") == 0)
                input = VIDEO;
            if (std::strcmp(argv[i+1], "IMG_SERIES") == 0)
                input = IMG_SERIES;
            if (std::strcmp(argv[i+1], "IMG_TIMESTAMP") == 0)
                input = IMG_TIMESTAMP;
            if (std::strcmp(argv[i+1], "ROS") == 0)
                input = ROS;
        }
    }

//     Ogre::LogManager * logManager = new Ogre::LogManager();
//     Log * log = LogManager::getSingleton().createLog("test.log", true, true, false);
    SVR* svr = new SVR( config_dialog, input );
    if (!svr->init())
    {
        delete svr;
        return 1;
    }
//     signal(SIGSEGV, &sigsegv_handler);

#ifdef USE_ROS
    ros::init(argc, argv, "svr_node");
    ros::NodeHandle mNh;

    svr->subscribeROSTopics(mNh);
#endif
    while (!svr->getQuit()
#ifdef USE_ROS
        && mNh.ok()
#endif
    )
    {
        svr->spin();
        switch(svr->mInputType)
        {
            case VIDEO:
                svr->updateVideoInput();
                break;
            case IMG_SERIES:
                svr->updateImgSeries();
                break;
            case IMG_TIMESTAMP:
                svr->updateImgTimestamp();
                break;
#ifdef USE_ROS
            case ROS:
                ros::spinOnce();
                break;
#else
            case ROS:
                svr->setQuit();
                LOG << "ROS is not available" << std::endl;
                break;
#endif
            default:
                svr->setQuit();
                LOG << "no input conigured" << std::endl;
                break;
        }
    }
    delete svr;
    return 0;
}
