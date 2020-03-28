#ifndef ROSSVR_H
#define ROSSVR_H

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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class ROSSVRNodelet : public nodelet::Nodelet
{
    public:
        ROSSVRNodelet();

        virtual void onInit();

        void newImageCallback(
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight
        );
        void newCameraInfoCallbackLeft(
            const sensor_msgs::CameraInfo::ConstPtr& input);
        void newCameraInfoCallbackRight(
            const sensor_msgs::CameraInfo::ConstPtr& input);

    private:
        SVRGameState *graphicsGameState;
        SVRGraphicsSystem *graphicsSystem;

        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeft;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRight;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;

        std::shared_ptr<ApproximateSync> mApproximateSync;

        Ogre::Window *mRenderWindow;
        Ogre::Timer mTimer;
        Ogre::uint64 mStartTime;
        Ogre::uint64 mTimeSinceLast;

        double accumulator = 1.0 / 60.0;


        double timeSinceLast = 1.0 / 60.0;
        void newCameraInfoCallback(
            const sensor_msgs::CameraInfo::ConstPtr& input,
            int leftOrRight);
};

#endif 
