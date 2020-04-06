#ifndef ROSSVR_H
#define ROSSVR_H

#include "SVR.h"

#include "OgreTimer.h"
#include "OgreWindow.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <thread>
#include <memory>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class ROSSVR : public nodelet::Nodelet
{
    public:
        ROSSVR();

        virtual void onInit();
        void mainloop();

        void newImageCallback(
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight
        );
        void newCameraInfoCallbackLeft(
            const sensor_msgs::CameraInfo::ConstPtr& input);
        void newCameraInfoCallbackRight(
            const sensor_msgs::CameraInfo::ConstPtr& input);

    private:
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeft;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRight;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;

        std::shared_ptr<ApproximateSync> mApproximateSync;

        volatile bool running_;               ///< device is running
        std::shared_ptr<Demo::SVR> mSVR;
        std::shared_ptr<std::thread> mSVRThread;

        bool mIsCameraInfoInit[2];
        void newCameraInfoCallback(
            const sensor_msgs::CameraInfo::ConstPtr& input,
            int leftOrRight);

};

#endif 
