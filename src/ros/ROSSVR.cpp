#include "SVR.h"

#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_calibration_parsers/parse_yml.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>



PLUGINLIB_EXPORT_CLASS( ROSSVRNodelet, nodelet::Nodelet )


ROSSVRNodelet::ROSSVRNodelet():
    graphicsGameState(0),
    graphicsSystem(0)

{
}

void ROSSVRNodelet::onInit()
{
    NODELET_DEBUG("Initializing ROSSVRNodelet...");
    ros::NodeHandle& nh = getNodeHandle();

    mSubImageLeft = new message_filters::Subscriber<PoseStamped>(
        nh, "left/image_rect", 20);
    mSubImageRight = new message_filters::Subscriber<PoseStamped>(
        nh, "rightimage_rect", 20);
    mSubCamInfo = nh.subscribe(
        "camera_info", 10, &ROSSVRNodelet::newCameraInfoCallback, this);

    mApproximateSync.reset(
        new ApproximateSync(ApproximatePolicy(queue_size),
        *mSubImageLeft, *mSubImageRight));
    mApproximateSync->registerCallback(std::bind(
        &ROSSVRNodelet::newImageCallback, this, _1, _2));

    ros::NodeHandle& nhPriv = getPrivateNodeHandle();

    try
    {
        graphicsSystem->initialize( getWindowTitle() );

        if( graphicsSystem->getQuit() )
        {
            graphicsSystem->deinitialize();

            destroySystems( graphicsGameState, graphicsSystem);

            return 0; //User cancelled config
        }

        Ogre::Window *renderWindow = graphicsSystem->getRenderWindow();

        graphicsSystem->createScene01();


    #if OGRE_USE_SDL2
        //Do this after creating the scene for easier the debugging (the mouse doesn't hide itself)
        SdlInputHandler *inputHandler = graphicsSystem->getInputHandler();
        inputHandler->setGrabMousePointer( true );
        inputHandler->setMouseVisible( false );
        inputHandler->setMouseRelative( true );
    #endif

        Ogre::Timer timer;
        Ogre::uint64 startTime = timer.getMicroseconds();
        double accumulator = 1.0 / 60.0;

        double timeSinceLast = 1.0 / 60.0;
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
            &(imgLeft->image), &(imgRight->image));
        graphicsSystem->update( static_cast<float>( timeSinceLast ) );
        graphicsSystem->getOvrCompositorListener()->setImgPtr(
            nullptr);
        graphicsSystem->finishFrameParallel();

        if( !renderWindow->isVisible() )
        {
            //Don't burn CPU cycles unnecessary when we're minimized.
            Ogre::Threads::Sleep( 500 );
        }

        Ogre::uint64 endTime = timer.getMicroseconds();
        timeSinceLast = (endTime - startTime) / 1000000.0;
        timeSinceLast = std::min( 1.0, timeSinceLast ); //Prevent from going haywire.
        accumulator += timeSinceLast;
        startTime = endTime;
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

void ROSSVRNodelet::newCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& camInfo )
{
    graphicsSystem->setCameraConfig(
        camInfo->width,
        camInfo->height,
        camInfo->K
    );
}


namespace nodelet_ROSSVR
{

class ROSSVR : public nodelet::Nodelet
{
public:

    ROSSVR() :
        graphicsGameState(0),
        graphicsSystem(0)
    {}

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    private_nh.getParam("value", value_);
    sub = private_nh.subscribe("resize", 10, &ROSSVR::callback, this);
    std_msgs::Float64Ptr
  }

  void callback(const std_msgs::Float64::ConstPtr& input)
  {
    std_msgs::Float64Ptr output(new std_msgs::Float64());
    output->data = input->data + value_;
    NODELET_DEBUG("Adding %f to get %f", value_, output->data);
    pub.publish(output);
  }

  ros::Publisher pub;
  ros::Subscriber sub;
  double value_;
};

PLUGINLIB_EXPORT_CLASS(nodelet_tutorial_math::ROSSVR, nodelet::Nodelet)
}
