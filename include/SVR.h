#ifndef _SVR_H_
#define _SVR_H_

#include "OgreWindow.h"
#include "OgreTimer.h"
#include "OgreCamera.h"

#include "opencv2/opencv.hpp"
#include <experimental/filesystem>

#ifdef USE_ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
#endif

namespace fs = std::experimental::filesystem;

#define LEFT 0
#define RIGHT 1
#define LOG std::cout
#define LOGEND std::endl
// #define LOG Ogre::LogManager::getSingleton().stream()
namespace Demo
{
    class SVRGameState;
    class SVRGraphicsSystem;

    typedef enum {
        NONE,
        ROS,
        VIDEO,
        IMG_SERIES,
        IMG_TIMESTAMP
    } InputType;
    typedef struct {
        Ogre::Matrix4 eyeToHead[2];
        Ogre::Matrix4 projectionMatrix[2];
        Ogre::Vector4 tan[2];
    } HmdConfig;
    typedef struct {
        int width[2] = {0,0};
        int height[2] = {0, 0};
        float f_x[2] = {0, 0};
        float f_y[2] = {0, 0};
        float c_x[2] = {0, 0};
        float c_y[2] = {0, 0};
    } CameraConfig;
    typedef struct {
        std::string path;
        cv::VideoCapture capture;
        int captureFrameWidth = 0;
        int captureFrameHeight = 0;
        int captureFramePixelFormat = 0;
    } VideoInput;
    typedef struct {
        int imgCnt;
        std::string path;
    } ImageSeriesInput;
    typedef struct {
        int imgCnt;
        std::string path;
    } ImageTimestampInput;

    class SVR {
        public:
            SVR(bool show_ogre_dialog, InputType inputType,
                VideoInput *videoInput,
                ImageSeriesInput *imageSeriesInput,
                ImageTimestampInput *imageTimestampInput);
            ~SVR();
            bool init();
            void spin(void);
            const char* getWindowTitle(void);
            bool getQuit();
            void setQuit();
            void initHmdConfig(HmdConfig *hmdConfig);
            void initCameraConfig(CameraConfig *cameraConfig);
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
            bool mQuit;
            bool mIsCameraInfoInit;
#ifdef USE_ROS
        private:
            message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeft;
            message_filters::Subscriber<sensor_msgs::Image>* mSubImageRight;
            ros::Subscriber mSubCamInfoLeft;
            ros::Subscriber mSubCamInfoRight;
            std::shared_ptr<ApproximateSync> mApproximateSync;
            void newROSCameraInfoCallback(
                const sensor_msgs::CameraInfo::ConstPtr& input,
                int leftOrRight);
        public:
            void subscribeROSTopics(ros::NodeHandle mNh);

            void newROSImageCallback(
                const sensor_msgs::Image::ConstPtr& imgLeft,
                const sensor_msgs::Image::ConstPtr& imgRight
            );
            void newROSCameraInfoCallbackLeft(
                const sensor_msgs::CameraInfo::ConstPtr& input);
            void newROSCameraInfoCallbackRight(
                const sensor_msgs::CameraInfo::ConstPtr& input);
#endif
    };
}

#endif
