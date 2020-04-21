#include "SVR.h"

#include "OgrePrerequisites.h"
#include <iostream>

#include "OgreCommon/SdlInputHandler.h"

#include "OgreCamera.h"
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

using namespace libconfig;
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
    SVR::SVR(bool show_ogre_dialog, InputType inputType,
        VideoInput *videoInput,
        ImageSeriesInput *imageSeriesInput,
        ImageTimestampInput *imageTimestampInput
    ) :
        mInputType(inputType),
        mVideoInput(videoInput),
        mImageSeriesInput(imageSeriesInput),
        mImageTimestampInput(imageTimestampInput),
        mQuit(false),
        mIsCameraInfoInit(false)
    {
        // Read the file. If there is an error, report it and exit.

        mGameState = new SVRGameState( "SVR" );

        mGraphicsSystem =
            new SVRGraphicsSystem( mGameState, show_ogre_dialog);

        mGameState->_notifyGraphicsSystem( mGraphicsSystem );

        mTimer = Ogre::Timer();
        mStartTime = mTimer.getMicroseconds();

        mAccumulator = 1.0 / 60.0;
        mTimeSinceLast = 1.0 / 60.0;

        switch(mInputType)
        {
            case VIDEO:
                initVideoInput();
                break;
            case IMG_SERIES:
                initImgSeries();
                break;
            case IMG_TIMESTAMP:
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
    }

    bool SVR::init()
    {
        try
        {
            mGraphicsSystem->initialize( getWindowTitle() );

            if( mGraphicsSystem->getQuit() )
            {
                mGraphicsSystem->deinitialize();

                LOG << "graphics System get quit" << LOGEND;
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
        return  mQuit || (mGraphicsSystem && mGraphicsSystem->getQuit());
    }

    void SVR::setQuit()
    {
        mQuit = true;
    }

    void SVR::initHmdConfig(HmdConfig *hmdConfig)
    {
        mGraphicsSystem->getOvrCompositorListener()->setHmdConfig(hmdConfig);
    }
    void SVR::initCameraConfig(CameraConfig *cameraConfig)
    {
        mGraphicsSystem->getOvrCompositorListener()->setCameraConfig(cameraConfig);
        mIsCameraInfoInit = true;
    }

    bool SVR::initVideoInput()
    {
        std::cout << mVideoInput->path << std::endl;
        mVideoInput->capture = VideoCapture(mVideoInput->path);
        mVideoInput->capture.set(CV_CAP_PROP_MODE,  CV_CAP_MODE_RGB );
        if (!mVideoInput->capture.isOpened()) {
            LOG << "Video could not be opened" <<LOGEND;
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
        mImageSeriesInput->imgCnt = 0;
        return true;
    }

    void SVR::updateImgSeries()
    {
        if (mImageSeriesInput->imgCnt >= 1000)
            mImageSeriesInput->imgCnt = 0;
        //TODO: change to left and right name
        std::stringstream ssl;
        ssl << mImageSeriesInput->path
            << std::setfill('0') << std::setw(3)
            << mImageSeriesInput->imgCnt
            << ".png";
        std::stringstream ssr;
        ssr << mImageSeriesInput->path
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
        mFileIteratorLeft = fs::directory_iterator(mImageTimestampInput->path);
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
        const sensor_msgs::CameraInfo::ConstPtr& camInfoLeft,
        const sensor_msgs::CameraInfo::ConstPtr& camInfoRight)
    {
        mIsCameraInfoInit = true;
        mGraphicsSystem->getOvrCompositorListener()->setCameraConfig(
            {
                CameraInfo(
                    camInfoLeft->width,
                    camInfoLeft->height,
                    camInfoLeft->K[0], camInfoLeft->K[4],
                    camInfoLeft->K[2], camInfoLeft->K[5]),
                CameraInfo(
                    camInfoRight->width,
                    camInfoRight->height,
                    camInfoRight->K[0], camInfoRight->K[4],
                    camInfoRight->K[2], camInfoRight->K[5])
            }
        );
    }
#endif
}


using namespace Demo;

InputType getInputType(std::string input_str)
{
    InputType input = NONE;
    if (input_str.compare("VIDEO") == 0)
        input = VIDEO;
    if (input_str.compare("IMG_SERIES") == 0)
        input = IMG_SERIES;
    if (input_str.compare("IMG_TIMESTAMP") == 0)
        input = IMG_TIMESTAMP;
    if (input_str.compare("ROS") == 0)
        input = ROS;
    return input;
}

int main( int argc, char *argv[] )
{
    bool show_ogre_dialog = false;
    InputType input = NONE;
    char *config_file = nullptr;
    CameraConfig *cameraConfig = nullptr;
    HmdConfig *hmdConfig = nullptr;
    VideoInput *videoInput = new VideoInput();
    ImageSeriesInput *imageSeriesInput = new ImageSeriesInput();
    ImageTimestampInput *imageTimestampInput = new ImageTimestampInput();

    for (int i = 1; i < argc; i++)
    {
        if (std::strcmp(argv[i], "--config") == 0 && i+1 < argc)
        {
            config_file = argv[i+1];
        }
        if (std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc)
        {
            input = getInputType(argv[i+1]);
        }
        if (std::strcmp(argv[i], "--video-path") == 0 && i+1 < argc)
        {
            videoInput->path = std::string(argv[i+1]);
            input = VIDEO;
        }
        if (std::strcmp(argv[i], "--img-series-path") == 0 && i+1 < argc)
        {
            imageSeriesInput->path = std::string(argv[i+1]);
            input = IMG_SERIES;
        }
        if (std::strcmp(argv[i], "--img-timestamp-path") == 0 && i+1 < argc)
        {
            imageTimestampInput->path = std::string(argv[i+1]);
            input = IMG_TIMESTAMP;
        }
        if(std::strcmp(argv[i], "--show-ogre-dialog") == 0)
            show_ogre_dialog = true;
    }

    if(config_file)
    {
        Config cfg;
        try
        {
            LOG << "read from config file" << LOGEND;
            cfg.readFile(config_file);
            if (cfg.exists("show_ogre_dialog"))
                cfg.lookupValue ("show_ogre_dialog", show_ogre_dialog);
            //only set input if we didn't set it by cmdline
            if (cfg.exists("input_type") && input == NONE)
            {
                std::string input_str;
                cfg.lookupValue("input_type", input_str);
                input = getInputType(input_str);
            }
            //VIDEO
            if (input == VIDEO && cfg.exists("video"))
            {
                Setting& vs = cfg.lookup("video");
                if (vs.exists("path"))
                    videoInput->path = vs["path"].c_str();;
                if (vs.exists("captureFrameWidth"))
                    videoInput->captureFrameWidth = 
                        vs["captureFrameWidth"];
                if (vs.exists("captureFrameHeight"))
                    videoInput->captureFrameHeight =
                        vs["captureFrameHeight"];
                if (vs.exists("captureFramePixelFormat"))
                    videoInput->captureFramePixelFormat =
                        vs["captureFramePixelFormat"];
            }
            //IMG_SERIES
            if (input == IMG_SERIES && cfg.exists("img_series.path"))
                cfg.lookupValue("img_series.path", imageSeriesInput->path);
            //IMG_TIMESTAMP
            if (input == IMG_SERIES && cfg.exists("img_timestamp.path"))
                cfg.lookupValue("img_timestamp.path", imageTimestampInput->path);

            std::string categories[2] = {"left", "right"};
            //CAMERA_CONFIG
            if (cfg.exists("camera_info.left") &&
                cfg.exists("camera_info.right"))
            {
                cameraConfig = new CameraConfig();
                const Setting& cis = cfg.lookup("camera_info");
                for (int leftOrRight = 0; leftOrRight < 2; leftOrRight++)
                {
                    Setting& s = cis.lookup(categories[leftOrRight]);
                    if (s.exists("width") && s.exists("height"))
                    {
                        //I'm not sure I have seen such a shit lib like libconfig++
                        //lookupValue gives me back 0 because it is parse point sth to 0 SHIT!!!!!
                        cameraConfig->width[leftOrRight] =
                            s["width"];
                        cameraConfig->height[leftOrRight] =
                            s["width"];
                    }
                    else
                        LOG << "camera_config is invalid" << LOGEND;
                    if (s.exists("f_x") && s.exists("f_y") &&
                        s.exists("c_x") && s.exists("c_y"))
                    {
                        cameraConfig->f_x[leftOrRight] = s["f_x"];
                        cameraConfig->f_y[leftOrRight] = s["f_y"];
                        cameraConfig->c_x[leftOrRight] = s["c_x"];
                        cameraConfig->c_y[leftOrRight] = s["c_y"];
                    }
                    if (s.exists("K") && s["K"].getLength() == 9)
                    {
                        cameraConfig->f_x[leftOrRight] = s["K"][0];
                        cameraConfig->f_y[leftOrRight] = s["K"][4];
                        cameraConfig->c_x[leftOrRight] = s["K"][2];
                        cameraConfig->c_y[leftOrRight] = s["K"][5];
                    }
                }
            }
            if (cfg.exists("hmd_info.left") &&
                cfg.exists("hmd_info.right"))
            {
                const Setting& cis = cfg.lookup("hmd_info");
                hmdConfig = new HmdConfig();
                for (int leftOrRight = 0; leftOrRight < 2; leftOrRight++)
                {
                    Setting& s = cis.lookup(categories[leftOrRight]);
                    if (s.exists("eye_to_head") &&
                        s["eye_to_head"].getLength() == 16 &&
                        s.exists("projection_matrix") &&
                        s["projection_matrix"].getLength() == 16 &&
                        s.exists("tan") &&
                        s["tan"].getLength() == 4)
                    {
                        Setting& st = s["tan"];
                        hmdConfig->tan[leftOrRight] =
                            Ogre::Vector4(st[0], st[1], st[2], st[3]);
                        Setting& se2h = s["eye_to_head"];
                        Setting& spm = s["projection_matrix"];
                        float e2h[16];
                        memset(&e2h, 0, sizeof(float[16]));
                        float pm[16];
                        memset(&pm, 0, sizeof(float[16]));
                        for (int i = 0; i < 16; i++)
                        {
                            //This is a horrible bug or feature of this library
                            //An array is not auto casting to float,
                            //an List can contain different types arrrrrg
                            // so we use lists... and casting
                            if (se2h[i].getType() == 1)
                            {
                                int a = se2h[i];
                                e2h[i] = static_cast<float>(a);
                            }
                            else
                            {
                                e2h[i] = se2h[i];
                            }
                            if (spm[i].getType() == 1)
                            {
                                int a = spm[i];
                                pm[i] = static_cast<float>(a);
                            }
                            else
                            {
                                pm[i] = spm[i];
                            }
                        }
                        hmdConfig->eyeToHead[leftOrRight] = Ogre::Matrix4(
                            e2h[ 0], e2h[ 1], e2h[ 2], e2h[ 3],
                            e2h[ 4], e2h[ 5], e2h[ 6], e2h[ 7],
                            e2h[ 8], e2h[ 9], e2h[10], e2h[11],
                            e2h[12], e2h[13], e2h[14], e2h[15]
                            );
                        hmdConfig->projectionMatrix[leftOrRight] = Ogre::Matrix4(
                            pm[ 0], pm[ 1], pm[ 2], pm[ 3],
                            pm[ 4], pm[ 5], pm[ 6], pm[ 7],
                            pm[ 8], pm[ 9], pm[10], pm[11],
                            pm[12], pm[13], pm[14], pm[15]
                        );
                    }
                    else
                    {
                        LOG << "There is some typo in your hmd config" << LOGEND;
                    }
                }
            }
        }
        catch(const FileIOException &fioex)
        {
            std::cerr << "I/O error while reading file." << std::endl;
            return(EXIT_FAILURE);
        }
        catch(const ParseException &pex)
        {
            std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                    << " - " << pex.getError() << std::endl;
            return(EXIT_FAILURE);
        }
    }

//     Ogre::LogManager * logManager = new Ogre::LogManager();
//     Log * log = LogManager::getSingleton().createLog("test.log", true, true, false);
    if (input == NONE)
    {
        LOG << "please configure output" << LOGEND;
        return 1;
    }

    SVR* svr = new SVR(show_ogre_dialog, input,
        videoInput,
        imageSeriesInput,
        imageTimestampInput
    );
    if (!svr->init())
    {
        delete svr;
        return 1;
    }
    if (hmdConfig)
    {
        svr->initHmdConfig(hmdConfig);
    }
    if (cameraConfig)
    {
        svr->initCameraConfig(cameraConfig);
    }
//     signal(SIGSEGV, &sigsegv_handler);

#ifdef USE_ROS
    ros::NodeHandle mNh;
    if(input == ROS)
    {
        ros::init(argc, argv, "svr_node");
        svr->subscribeROSTopics(mNh);
    }
#endif
    while (!svr->getQuit()
#ifdef USE_ROS
        && (input != ROS || mNh.ok())
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

    //maybe save changes in hmdConfig;
    delete cameraConfig;
    delete hmdConfig;
    delete svr;
    delete videoInput;
    delete imageSeriesInput;
    delete imageTimestampInput;
    return 0;
}
