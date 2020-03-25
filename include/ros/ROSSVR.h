#ifndef ROSSVR_H
#define ROSSVR_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

typedef sync_policies::ApproximateTime<PoseStamped, PointCloud2> ApproximatePolicy;
typedef Synchronizer<ApproximatePolicy> ApproximateSync;

class ROSSVRNodelet : public nodelet::Nodelet
{
    public:
        ROSSVRNodelet() {};

        virtual void onInit();

        void newImageCallback(
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgLeft
        );
        void newCameraInfoCallback(
            const sensor_msgs::CameraInfo::ConstPtr& input);

    private:
        SVRGameState *graphicsGameState;
        SVRGraphicsSystem *graphicsSystem;

        Subscriber<sensor_msgs::Image>* mSubImageLeft;
        Subscriber<sensor_msgs::Image>* mSubImageRight;
        ros::Subscriber subCamInfo;

        std::shared_ptr<ApproximateSync> approximate_sync_;
};

#endif //UNDISTORT_NODELET_H
