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
        mImageTimestampInput(nullptr)
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
        }
    }

    SVR::~SVR()
    {
        mGraphicsSystem->deinitialize();
        delete mGraphicsSystem;
        delete mGameState;
        if (mVideoInput)
            delete mVideoInput;
//         if (mImageSeriesInput)
//             delete mImageSeriesInput;
//         if (mImageTimestampInput)
//             delete mImageTimestampInput;
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

    void SVR::spin() {
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
        return mGraphicsSystem && mGraphicsSystem->getQuit();
    }

    bool SVR::initVideoInput() {
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
}

using namespace Demo;
int main( int argc, const char *argv[] )
{
    bool config_dialog = false;
    InputType input = VIDEO;
    for (int i = 1; i < argc; i++)
    {
        config_dialog = config_dialog || std::strcmp(argv[i], "--config-dialog") == 0;
        if (std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc)
        {
            if (std::strcmp(argv[i], "VIDEO") == 0)
                input = VIDEO;
            if (std::strcmp(argv[i], "IMG_SERIES") == 0)
                input = IMG_SERIES;
            if (std::strcmp(argv[i], "IMG_TIMESTAMP") == 0)
                input = IMG_TIMESTAMP;
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
    while (!svr->getQuit())
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
        }
    }
    delete svr;
    return 0;
}
