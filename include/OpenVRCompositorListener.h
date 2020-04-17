#ifndef _Demo_OpenVRCompositorListener_H_
#define _Demo_OpenVRCompositorListener_H_

#include "SVR.h"

#include "OgreFrameListener.h"
#include "Compositor/OgreCompositorWorkspaceListener.h"
#include "OgreStagingTexture.h"

#include "OgreMatrix4.h"
#include "OgreCamera.h"

#include "opencv2/opencv.hpp"

#include <experimental/filesystem>
#include <mutex>

#if __cplusplus <= 199711L
    #ifndef nullptr
        #define OgreDemoNullptrDefined
        #define nullptr (0)
    #endif
#endif
#include "openvr.h"
#if __cplusplus <= 199711L
    #ifdef OgreDemoNullptrDefined
        #undef OgreDemoNullptrDefined
        #undef nullptr
    #endif
#endif

namespace fs = std::experimental::filesystem;

namespace Demo
{
    namespace VrWaitingMode
    {
        enum VrWaitingMode
        {
            AfterSwap,
            BeforeSceneGraph,
            AfterSceneGraph,
            BeforeShadowmaps,
            BeforeFrustumCulling,
            AfterFrustumCulling,
            NumVrWaitingModes
        };
    }

    class OpenVRCompositorListener : public Ogre::FrameListener, public Ogre::CompositorWorkspaceListener
    {
    protected:
        vr::IVRSystem		*mHMD;
        vr::IVRCompositor	*mVrCompositor3D;

        Ogre::TextureGpu	*mVrTexture;
        Ogre::Root          *mRoot;
        Ogre::RenderSystem  *mRenderSystem;

        Ogre::CompositorWorkspace *mWorkspace;

        int mValidPoseCount;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
        Ogre::Matrix4           mDevicePose[vr::k_unMaxTrackedDeviceCount];
        vr::ETextureType        mApiTextureType;
        Ogre::VrData            mVrData;
        Ogre::Camera            *mCamera;
        Ogre::Camera            *mVrCullCamera;
        Ogre::Vector3           mCullCameraOffset;

        VrWaitingMode::VrWaitingMode mWaitingMode;
        VrWaitingMode::VrWaitingMode mFirstGlitchFreeMode;
        Ogre::Real              mLastCamNear;
        Ogre::Real              mLastCamFar;
        bool                    mMustSyncAtEndOfFrame;

        static Ogre::Matrix4 convertSteamVRMatrixToMatrix4( vr::HmdMatrix34_t matPose );
        static Ogre::Matrix4 convertSteamVRMatrixToMatrix4( vr::HmdMatrix44_t matPose );
        void updateHmdTrackingPose(void);

        void syncCullCamera(void);
        void syncCamera(void);
        void syncCameraProjection( bool bForceUpdate );
        void syncCameraProjectionNoHMD( bool bForceUpdate );

        void setupImageData();

        //left_left, left_right right_left right_right
        //left_top left_bottom  right_top right_bottom
        struct Align {
            int leftLeft;
            int leftTop;
            int rightLeft;
            int rightTop;
        } mAlign;
        struct CameraConfig {
            float width;
            float height;
            float f_x;
            float f_y;
            float c_x;
            float c_y;
        } mCameraConfig[2];
        size_t mImgWidthOrig;
        cv::Size mImageResizeSize[2];
        int mCVr[4];
        int mImgMiddleResize[4];

        std::mutex mMtxImageResize;
        cv::Mat mImageResize[2];
        Ogre::uint8 *mImageData;
        Ogre::StagingTexture *mStagingTexture;

        int mRefreshFrameNum;
        int mFrameCnt;
        bool calcAlign();

        bool fillTexture(void);
        bool clearTexture(void);
        bool mWriteTexture;
        bool mShowMovie;

    public:
        OpenVRCompositorListener(
            vr::IVRSystem *hmd, vr::IVRCompositor *vrCompositor,
            Ogre::TextureGpu *vrTexture, Ogre::Root *root,
            Ogre::CompositorWorkspace *workspace,
            Ogre::Camera *camera, Ogre::Camera *cullCamera,
            int refreshFrameNum );
        virtual ~OpenVRCompositorListener();

        virtual bool frameStarted( const Ogre::FrameEvent& evt );
        virtual bool frameRenderingQueued( const Ogre::FrameEvent &evt );
        virtual bool frameEnded( const Ogre::FrameEvent& evt );

        virtual void workspacePreUpdate( Ogre::CompositorWorkspace *workspace );
        virtual void passPreExecute( Ogre::CompositorPass *pass );

        virtual void passSceneAfterShadowMaps( Ogre::CompositorPassScene *pass );
        virtual void passSceneAfterFrustumCulling( Ogre::CompositorPassScene *pass );

        /// See VrWaitingMode::VrWaitingMode
        void setWaitingMode( VrWaitingMode::VrWaitingMode waitingMode );
        VrWaitingMode::VrWaitingMode getWaitingMode(void)   { return mWaitingMode; }

        InputType getInputType();
        void setInputType(InputType);
        void showMovie()
        {
            mShowMovie = !mShowMovie;
            mFrameCnt = 3;
        };
        void triggerWriteTexture()
        {mWriteTexture = true;};
        float getImgScale();
        void setImgScale(float imgScale);
        void setImgPtr(const cv::Mat *left, const cv::Mat *right);
        void setCameraConfig(
            float width, float height,
            float f_x, float f_y,
            float c_x, float c_y,
            int leftOrRightCam);

        /** When operating in VrWaitingMode::AfterSceneGraph or later, there's a chance
            graphical artifacts appear if the camera transform is immediately changed after
            calling WaitGetPoses instead of waiting for the next frame.

            This is a question of whether you want to prioritize latency over graphical
            artifacts, or an artifact-free rendering in exchange for up to one frame of
            latency. The severity and frequency of artifacts may vastly depend on the game.

            Because the graphical artifacts get more severe with each mode, you may e.g.
            find acceptable to tolerate artifacts in VrWaitingMode::BeforeFrustumCulling,
            but not in VrWaitingMode::AfterFrustumCulling
        @param glitchFree
            First stage to consider glitch-free. e.g. calling:
                setGlitchFree( VrWaitingMode::AfterSwap );
            Means that all waiting modes must behave glitch-free, while calling:
                setGlitchFree( VrWaitingMode::BeforeSceneGraph );
            Means that only AfterSwap is allowed to have glitches, while:
                setGlitchFree( VrWaitingMode::NumVrWaitingModes );
            Means that any waiting mode is allowed to have glitches
        */
        void setGlitchFree( VrWaitingMode::VrWaitingMode firstGlitchFreeMode );

        /** Returns true if the current waiting mode can update the camera immediately,
            false if it must wait until the end of the frame.
        @remarks
            VrWaitingMode::BeforeSceneGraph and earlier always returns true.

            See OpenVRCompositorListener::setGlitchFree
        */
        bool canSyncCameraTransformImmediately(void) const;

    };
}

#endif
