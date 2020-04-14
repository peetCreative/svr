#ifndef _SVR_H_
#define _SVR_H_

#include "OgreWindow.h"
#include "OgreTimer.h"

#include "opencv2/opencv.hpp"
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

#define LEFT 0
#define RIGHT 1
#define LOG std::cout
// #define LOG Ogre::LogManager::getSingleton().stream()

namespace Demo
{
    class SVRGameState;
    class SVRGraphicsSystem;

    typedef enum {
        VIDEO,
        IMG_SERIES,
        IMG_TIMESTAMP
    } InputType;

    typedef struct {
        std::string videoPath;
        cv::VideoCapture capture;
        int captureFrameWidth;
        int captureFrameHeight;
        int captureFramePixelFormat;
    } VideoInput;
    typedef struct {
        int imgCnt;
        std::string imgPath;
    } ImageSeriesInput;
    typedef struct {
        int imgCnt;
        std::string imgPath;
    } ImageTimestampInput;

    class SVR {
        public:
            SVR(bool showConfigDialog, InputType inputType );
            ~SVR();
            bool init();
            void spin(void);
            const char* getWindowTitle(void);
            bool getQuit();
            bool initVideoInput();
            void updateVideoInput();
            bool initImgSeries();
            void updateImgSeries();
            bool initImgTimestamp();
            void updateImgTimestamp();
            const InputType mInputType;

            SVRGameState *getGameState()
            {
                return mGameState;
            };
            SVRGraphicsSystem *getGraphicsSystem()
            {
                return mGraphicsSystem;
            };
        private:
            SVRGameState *mGameState;
            SVRGraphicsSystem *mGraphicsSystem;

            Ogre::Window *mRenderWindow;
            Ogre::Timer mTimer;
            Ogre::uint64 mStartTime;

            bool mIsCameraInfoInit[2];

            double mAccumulator = 1.0 / 60.0;
            double mTimeSinceLast = 1.0 / 60.0;

            //of vr height
            float mImgScale;
            //cols/rows
            float mImgRatio;

            cv::Mat* mImageOrig[2];

            VideoInput *mVideoInput;
            ImageSeriesInput *mImageSeriesInput;
            ImageTimestampInput *mImageTimestampInput;
            int mCaptureFrameWidth, mCaptureFrameHeight,
                mCaptureFramePixelFormat;
            fs::directory_iterator mFileIteratorLeft;

    };
}

#endif
