
#include "OpenVRCompositorListener.h"
#include "SVR.h"

#include "OgreTextureGpuManager.h"
#include "OgrePixelFormatGpuUtils.h"
#include "OgreMemoryAllocatorConfig.h"
#include "OgreGpuResource.h"
#include "OgreStagingTexture.h"

#include "OgreTextureGpu.h"
#include "OgreRenderSystem.h"
#include "Compositor/Pass/OgreCompositorPass.h"
#include "Compositor/Pass/OgreCompositorPassDef.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassScene.h"

#include "Compositor/OgreCompositorWorkspace.h"

#include "opencv2/opencv.hpp"
#include <sstream>
#include <cmath>
#include <mutex>

using namespace Ogre;
using namespace cv;
namespace fs = std::experimental::filesystem;

namespace Demo
{
    OpenVRCompositorListener::OpenVRCompositorListener(
            vr::IVRSystem *hmd, vr::IVRCompositor *vrCompositor3D,
            Ogre::TextureGpu *vrTexture, Ogre::Root *root,
            Ogre::CompositorWorkspace *workspace,
            Ogre::Camera *camera, Ogre::Camera *cullCamera,
            int refreshFrameNum ) :
        mHMD( hmd ),
        mVrCompositor3D( vrCompositor3D ),
        mVrTexture( vrTexture ),
        mRoot( root ),
        mRenderSystem( root->getRenderSystem() ),
        mWorkspace( workspace ),
        mValidPoseCount( 0 ),
        mApiTextureType( vr::TextureType_Invalid ),
        mCamera( camera ),
        mVrCullCamera( cullCamera ),
        mCullCameraOffset( Ogre::Vector3::ZERO ),
        mWaitingMode( VrWaitingMode::BeforeSceneGraph ),
        mFirstGlitchFreeMode( VrWaitingMode::NumVrWaitingModes ),
        mLastCamNear( 0 ),
        mLastCamFar( 0 ),
        mMustSyncAtEndOfFrame( false ),
        mImageResizeSize{Size(0,0), Size(0,0)},
        mCVr{0,0,0,0},
        mFrameCnt(0),
        mRefreshFrameNum(refreshFrameNum),
        mWriteTexture(true),
        mShowMovie(true)
    {
        memset( mDevicePose, 0, sizeof( mDevicePose ) );
        memset( &mVrData, 0, sizeof( mVrData ) );
        mAlign.leftLeft = 0;
        mAlign.leftTop = 0;
        mAlign.rightLeft = 0;
        mAlign.rightTop = 0;

        //TODO: read the correct camera calibration info
        //DUMMY CAMERACALIBRATION READ CORRECT ONE
        mCameraConfig[LEFT].width = 640;
        mCameraConfig[LEFT].height = 512;
        mCameraConfig[LEFT].f_x = 534.195;
        mCameraConfig[LEFT].f_y = 534.095;
        mCameraConfig[LEFT].c_x = 300.45;
        mCameraConfig[LEFT].c_y = 250.37;
        mCameraConfig[RIGHT].width = 640;
        mCameraConfig[RIGHT].height = 512;
        mCameraConfig[RIGHT].f_x = 533.915;
        mCameraConfig[RIGHT].f_y = 533.8;
        mCameraConfig[RIGHT].c_x = 349.11;
        mCameraConfig[RIGHT].c_y = 250.825;

        if(mHMD)
        {
            mCamera->setVrData( &mVrData );
            syncCameraProjection( true );
            LOG << "camera left to right " << mVrData.mLeftToRight.x <<
                " "  << mVrData.mLeftToRight.y  << std::endl;
            float left, right, top, bottom;
            mHMD->GetProjectionRaw(vr::Eye_Left, &left, &right, &top, &bottom);
            LOG << "left camera tan left and right " << left << " " << right << std::endl;
            LOG << "left camera tan top and bottom " << top << " " << bottom << std::endl;
            mHMD->GetProjectionRaw(vr::Eye_Right, &left, &right, &top, &bottom);
            LOG << "right camera tan left and right " << left << " " << right << std::endl;
            LOG << "right camera tan top and bottom " << top << " " << bottom << std::endl;
        }
        else
        {
            LOG << "Sync Camera Projection No HMD" << std::endl;
            syncCameraProjectionNoHMD( true );
        }
        calcAlign();

        mRoot->addFrameListener( this );
        mWorkspace->setListener( this );

        const Ogre::String &renderSystemName = mRenderSystem->getName();
        if( renderSystemName == "OpenGL 3+ Rendering Subsystem" )
            mApiTextureType = vr::TextureType_OpenGL;

        setupImageData();
    }

    //-------------------------------------------------------------------------
    OpenVRCompositorListener::~OpenVRCompositorListener()
    {
        TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();
        //Tell the TextureGpuManager we're done with this StagingTexture. Otherwise it will leak.
        textureManager->removeStagingTexture( mStagingTexture );
        mStagingTexture = 0;
        //Do not free the pointer if texture's paging strategy is GpuPageOutStrategy::AlwaysKeepSystemRamCopy
        OGRE_FREE_SIMD( mImageData, MEMCATEGORY_RESOURCE );
        mImageData = 0;
        if( mCamera )
            mCamera->setVrData( 0 );

        if( mWorkspace->getListener() == this )
            mWorkspace->setListener( 0 );
        mRoot->removeFrameListener( this );
    }

    void OpenVRCompositorListener::setupImageData()
    {
        TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();
        const uint32 rowAlignment = 4u;
        const size_t dataSize =
            PixelFormatGpuUtils::getSizeBytes(
                mVrTexture->getWidth(),
                mVrTexture->getHeight(),
                mVrTexture->getDepth(),
                mVrTexture->getNumSlices(),
                mVrTexture->getPixelFormat(),
                rowAlignment );

        mImageData = reinterpret_cast<uint8*>(
            OGRE_MALLOC_SIMD( dataSize,
            MEMCATEGORY_RESOURCE ) );
        memset(mImageData, 0, dataSize);

        //We have to upload the data via a StagingTexture, which acts as an intermediate stash
        //memory that is both visible to CPU and GPU.
        mStagingTexture = 
            textureManager->getStagingTexture(
                mVrTexture->getWidth(),
                mVrTexture->getHeight(),
                mVrTexture->getDepth(),
                mVrTexture->getNumSlices(),
                mVrTexture->getPixelFormat() );

    }
    //-------------------------------------------------------------------------
    Ogre::Matrix4 OpenVRCompositorListener::convertSteamVRMatrixToMatrix4( vr::HmdMatrix34_t matPose )
    {
        Ogre::Matrix4 matrixObj(
                    matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
                    matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
                    matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
                               0.0f,            0.0f,            0.0f,            1.0f );
        return matrixObj;
    }
    //-------------------------------------------------------------------------
    Ogre::Matrix4 OpenVRCompositorListener::convertSteamVRMatrixToMatrix4( vr::HmdMatrix44_t matPose )
    {
        Ogre::Matrix4 matrixObj(
                    matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
                    matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
                    matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
                    matPose.m[3][0], matPose.m[3][1], matPose.m[3][2], matPose.m[3][3] );
        return matrixObj;
    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::syncCameraProjectionNoHMD( bool bForceUpdate )
    {
        const Ogre::Real camNear = mCamera->getNearClipDistance();
        const Ogre::Real camFar = mCamera->getFarClipDistance();

        if( mLastCamNear != camNear || mLastCamFar != camFar || bForceUpdate )
        {
            Ogre::Matrix4 eyeToHead[2] = { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY };
            Ogre::Matrix4 projectionMatrixRS[2] = { mCamera->getProjectionMatrixWithRSDepth(),
                                                    mCamera->getProjectionMatrixWithRSDepth() };

            mVrData.set( eyeToHead, projectionMatrixRS );
            mLastCamNear = camNear;
            mLastCamFar = camFar;

            mVrCullCamera->setNearClipDistance( camNear );
            mVrCullCamera->setFarClipDistance( camFar );
            mVrCullCamera->setFOVy( mCamera->getFOVy() );
        }
    }

    inline void printMatrix4(Ogre::Matrix4 m)
    {
        LOG << m[0][0] << " "
            << m[0][1] << " "
            << m[0][2] << " "
            << m[0][3] << " " << std::endl
            << m[1][0] << " "
            << m[1][1] << " "
            << m[1][2] << " "
            << m[1][3] << " " << std::endl
            << m[2][0] << " "
            << m[2][1] << " "
            << m[2][2] << " "
            << m[2][3] << " " << std::endl
            << m[3][0] << " "
            << m[3][1] << " "
            << m[3][2] << " "
            << m[3][3] << " " << std::endl;

    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::syncCameraProjection( bool bForceUpdate )
    {
        const Ogre::Real camNear = mCamera->getNearClipDistance();
        const Ogre::Real camFar  = mCamera->getFarClipDistance();

        if( mLastCamNear != camNear || mLastCamFar != camFar || bForceUpdate )
        {
            Ogre::Matrix4 eyeToHead[2];
            Ogre::Matrix4 projectionMatrix[2];
            Ogre::Matrix4 projectionMatrixRS[2];
            Ogre::Vector4 eyeFrustumExtents[2];

            for( size_t i=0u; i<2u; ++i )
            {
                vr::EVREye eyeIdx = static_cast<vr::EVREye>( i );
                eyeToHead[i] = convertSteamVRMatrixToMatrix4( mHMD->GetEyeToHeadTransform( eyeIdx ) );
                projectionMatrix[i] =
                        convertSteamVRMatrixToMatrix4( mHMD->GetProjectionMatrix( eyeIdx,
                                                                                  camNear, camFar ) );
                mRenderSystem->_convertOpenVrProjectionMatrix( projectionMatrix[i],
                                                               projectionMatrixRS[i] );
                mHMD->GetProjectionRaw(
                    eyeIdx,
                    &eyeFrustumExtents[i].x, &eyeFrustumExtents[i].y,
                    &eyeFrustumExtents[i].z, &eyeFrustumExtents[i].w );
                LOG<< "eyeToHead"<< std::endl;
                printMatrix4(eyeToHead[i]);
                LOG<< "projectionMatrix"<< std::endl;
                printMatrix4(projectionMatrix[i]);
                LOG<< "projectionMatrixRS"<< std::endl;
                printMatrix4(projectionMatrixRS[i]);
            }

            mVrData.set( eyeToHead, projectionMatrixRS );
            mLastCamNear = camNear;
            mLastCamFar = camFar;

            Ogre::Vector4 cameraCullFrustumExtents;
            cameraCullFrustumExtents.x = std::min(
                eyeFrustumExtents[0].x, eyeFrustumExtents[1].x );
            cameraCullFrustumExtents.y = std::max(
                eyeFrustumExtents[0].y, eyeFrustumExtents[1].y );
            cameraCullFrustumExtents.z = std::max(
                eyeFrustumExtents[0].z, eyeFrustumExtents[1].z );
            cameraCullFrustumExtents.w = std::min(
                eyeFrustumExtents[0].w, eyeFrustumExtents[1].w );

            mVrCullCamera->setFrustumExtents(
                cameraCullFrustumExtents.x,
                cameraCullFrustumExtents.y,
                cameraCullFrustumExtents.w,
                cameraCullFrustumExtents.z,
                Ogre::FET_TAN_HALF_ANGLES );

            const float ipd = mVrData.mLeftToRight.x;
            mCullCameraOffset = Ogre::Vector3::ZERO;
            mCullCameraOffset.z = (ipd / 2.0f) / Ogre::Math::Abs( cameraCullFrustumExtents.x );

            const Ogre::Real offset = mCullCameraOffset.length();
            mVrCullCamera->setNearClipDistance( camNear + offset );
            mVrCullCamera->setFarClipDistance( camFar + offset );
        }
    }

    bool OpenVRCompositorListener::calcAlign()
    {
        //now we have to know
        size_t vr_width_half = mVrTexture->getWidth()/2;

        float tan[4][2];
        //leftLeft, rightLeft leftTop rightTop
        size_t align[4];
        size_t vr_size[4] = {vr_width_half, vr_width_half,
            mVrTexture->getHeight(), mVrTexture->getHeight()};
        float img_size[4] = {
            mCameraConfig[LEFT].width,
            mCameraConfig[RIGHT].width,
            mCameraConfig[LEFT].height,
            mCameraConfig[RIGHT].height};
        float f_cam[4] = {
            mCameraConfig[LEFT].f_x,
            mCameraConfig[RIGHT].f_x,
            mCameraConfig[LEFT].f_y,
            mCameraConfig[RIGHT].f_y};
        float c_cam[4] = {
            mCameraConfig[LEFT].c_x,
            mCameraConfig[RIGHT].c_x,
            mCameraConfig[LEFT].c_y,
            mCameraConfig[RIGHT].c_y};

        if (mHMD)
        {
            mHMD->GetProjectionRaw(
                vr::Eye_Left, &tan[0][0], &tan[0][1], &tan[2][0], &tan[2][1]);
            mHMD->GetProjectionRaw(
                vr::Eye_Right, &tan[1][0], &tan[1][1], &tan[3][0], &tan[3][1]);
        }
        else
        {
            // left left
                tan[0][0] = -1.39377;
            // left right
                tan[0][1] = 1.23437;
            // left top
                tan[2][0] = -1.46653 ;
            // left bottom
                tan[2][1] = 1.45802 ;
            // right left
                tan[1][0] = -1.24354 ;
            // right right
                tan[1][1] = 1.39482;
            // right top
                tan[3][0] = -1.47209;
            // right bottom
                tan[3][1] = 1.45965;
        }
/*
        LOG << "ratio left: " << vr_width_half << " " << mVrTexture->getHeight() << std::endl;
        LOG << "tan: "
            << tan[0][0] << " " << tan[0][1] << " "
            << tan[1][0] << " " << tan[1][1] << " "
            << tan[2][0] << " " << tan[2][1] << " "
            << tan[3][0] << " " << tan[3][1] << std::endl;*/

        for (int i = 0; i < 4; i++)
        {
            float c_vr = -tan[i][0] * vr_size[i] / (-tan[i][0] + tan[i][1]);
            float img_size_resize = c_vr * img_size[i] / (f_cam[i] *-tan[i][0]);
            float img_middle_resize = img_size_resize * c_cam[i] / img_size[i];
            float align_f = c_vr - img_middle_resize;
            if (align_f <= 0)
                return false;
            align[i] = static_cast<size_t>(std::round(align_f));
            OGRE_ASSERT_MSG(img_size_resize > 0, "img_height_resize smaller than zero");
            if (i < 2) {
                mImageResizeSize[i%2].width = static_cast<size_t>(
                    std::round(img_size_resize));
            } else {
                mImageResizeSize[i%2].height = static_cast<size_t>(
                    std::round(img_size_resize));
            }
            mCVr[i] = static_cast<size_t>(std::round(c_vr));
            mImgMiddleResize[i] = static_cast<size_t>(
                std::round(img_middle_resize));
        }

        mImageResize[LEFT] = Mat();
        mImageResize[RIGHT] = Mat();

        LOG << "left resize: " << mImageResizeSize[LEFT].width
            << " " << mImageResizeSize[LEFT].height << std::endl;
        LOG << "right resize: " << mImageResizeSize[RIGHT].width 
            << " " << mImageResizeSize[RIGHT].height << std::endl;

        mAlign.leftLeft = align[0];
        mAlign.rightLeft = vr_width_half + align[1];
        mAlign.leftTop = align[2];
        mAlign.rightTop = align[3];


        LOG << "Align leftLeft:" << mAlign.leftLeft << std::endl;
        LOG << "Align leftTop:" << mAlign.leftTop << std::endl;
        LOG << "Align rightLeft:" << mAlign.rightLeft << std::endl;
        LOG << "Align rightTop:" << mAlign.rightTop << std::endl;
        return true;
    }
    bool OpenVRCompositorListener::fillTexture(void)
    {
        const size_t bytesPerPixel = 4u;
        const size_t bytesPerRow =
            mVrTexture->_getSysRamCopyBytesPerRow( 0 );

//         LOG << mVrTexture->getWidth() << std::endl;
//         LOG << mVrTexture->getHeight() << std::endl;
//         LOG << width_resize << std::endl;
//         LOG << "height_resize1 " << height_resize << std::endl;
//         LOG << height_resize << std::endl;

//         LOG << "width_resize " << width_resize << std::endl;
//         LOG << "height_resize " << height_resize << std::endl;
//         LOG << "ldst.cols " << ldst.cols << std::endl;
//         LOG << "ldst.rows " << ldst.rows << std::endl;

        if ( !mImageResize[LEFT].empty() && !mImageResize[RIGHT].empty()) {
            size_t align_left;
            size_t align_top;
            Mat* dst;
            for(size_t i = 0; i < 2u; i++) {
                if (i == 0) {
                    align_left = mAlign.leftLeft;
                    align_top = mAlign.leftTop;
                    dst = &mImageResize[LEFT];
                }
                else {
                    align_left =  mAlign.rightLeft;
                    align_top = mAlign.rightTop;
                    dst = &mImageResize[RIGHT];
                }
                size_t row_cnt = align_top * bytesPerRow;
                for (size_t y = 0; y < mImageResizeSize[i].height; y++) {
                    size_t cnt = row_cnt + (align_left * bytesPerPixel);
                    uint8_t* img_row_ptr = dst->ptr<uint8_t>(y);
                    for (size_t x = 0; x < mImageResizeSize[i].width; x++) {
                        mImageData[cnt++] = *(img_row_ptr+2);
                        mImageData[cnt++] = *(img_row_ptr+1);
                        mImageData[cnt++] = *img_row_ptr;
                        img_row_ptr += 3;
                        mImageData[cnt++] = 0;
                    }
                    row_cnt += bytesPerRow;
                }
            }
        }

        //redline for eye pupilar middle
        for (size_t i = 0; i < mVrTexture->getHeight(); i++)
        {
            mImageData[(bytesPerRow*i) + (mCVr[0] * bytesPerPixel)] = 255;
            mImageData[(bytesPerRow*i) + (bytesPerRow/2) + (mCVr[1] * bytesPerPixel)+4] = 255;
        }

        //green line for middle
        for (size_t i = 0; i < mVrTexture->getHeight()*4; i++)
        {
            mImageData[(bytesPerRow/4*i)+1] = 255;
        }
//         mVrTexture->_transitionTo( GpuResidency::Resident, imageData );
        mVrTexture->_setNextResidencyStatus( GpuResidency::Resident );

        mStagingTexture->startMapRegion();
        TextureBox texBox = mStagingTexture->mapRegion(
            mVrTexture->getWidth(),
            mVrTexture->getHeight(),
            mVrTexture->getDepth(),
            mVrTexture->getNumSlices(),
            mVrTexture->getPixelFormat() );
        texBox.copyFrom( mImageData,
                         mVrTexture->getWidth(),
                         mVrTexture->getHeight(),
                         bytesPerRow );
        mStagingTexture->stopMapRegion();
        mStagingTexture->upload( texBox, mVrTexture, 0, 0, 0, false );

        mVrTexture->notifyDataIsReady();
        return true;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::clearTexture(void)
    {
        const size_t bytesPerRow =
            mVrTexture->_getSysRamCopyBytesPerRow( 0 );

        const uint32 rowAlignment = 4u;
        const size_t dataSize =
            PixelFormatGpuUtils::getSizeBytes(
                mVrTexture->getWidth(),
                mVrTexture->getHeight(),
                mVrTexture->getDepth(),
                mVrTexture->getNumSlices(),
                mVrTexture->getPixelFormat(),
                rowAlignment );
        memset(mImageData, 0, dataSize);
        mStagingTexture->startMapRegion();
        TextureBox texBox = mStagingTexture->mapRegion(
            mVrTexture->getWidth(),
            mVrTexture->getHeight(),
            mVrTexture->getDepth(),
            mVrTexture->getNumSlices(),
            mVrTexture->getPixelFormat() );
        texBox.copyFrom( mImageData,
                         mVrTexture->getWidth(),
                         mVrTexture->getHeight(),
                         bytesPerRow );
        mStagingTexture->stopMapRegion();
        mStagingTexture->upload( texBox, mVrTexture, 0, 0, 0, false );

        mVrTexture->notifyDataIsReady();
        return true;
    }
    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameStarted( const Ogre::FrameEvent& evt )
    {
        if ( mHMD )
            mVrCompositor3D->WaitGetPoses( mTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

        if( mShowMovie )
        {
            if (mFrameCnt-- == 0 )
            {
                fillTexture();
                mFrameCnt = mRefreshFrameNum;
            }
        }
        else
        {
            if (mFrameCnt != 0 )
            {
                clearTexture();
                mFrameCnt = 0;
            }
        }

        return true;
    }


    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameRenderingQueued( const Ogre::FrameEvent &evt )
    {
        vr::VRTextureBounds_t texBounds;
        if( mVrTexture->requiresTextureFlipping() )
        {
            texBounds.vMin = 1.0f;
            texBounds.vMax = 0.0f;
        }
        else
        {
            texBounds.vMin = 0.0f;
            texBounds.vMax = 1.0f;
        }

        vr::Texture_t eyeTexture =
        {
            0,
            mApiTextureType,
            vr::ColorSpace_Gamma
        };
        if (mHMD)
        {
            if (mWriteTexture) {
                mVrTexture->writeContentsToFile("texture.bmp", 8, 255);
                mWriteTexture = false;
            }
        }

        TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();
        textureManager->waitForStreamingCompletion();
        mVrTexture->waitForData();
        mVrTexture->getCustomAttribute(
            Ogre::TextureGpu::msFinalTextureBuffer,
            &eyeTexture.handle );

        if (mHMD)
        {
            texBounds.uMin = 0;
            texBounds.uMax = 0.5f;
            mVrCompositor3D->Submit( vr::Eye_Left, &eyeTexture, &texBounds );
            texBounds.uMin = 0.5f;
            texBounds.uMax = 1.0f;
            mVrCompositor3D->Submit( vr::Eye_Right, &eyeTexture, &texBounds );
        }

        mRenderSystem->flushCommands();

        return true;
    }

    void OpenVRCompositorListener::setImgPtr(const cv::Mat *left, const cv::Mat *right)
    {
//         LOG << mImageResizeSize[LEFT].width << std::endl;
//         LOG << mImageResizeSize[LEFT].height << std::endl;
//         LOG << mImageResizeSize[RIGHT].width << std::endl;
//         LOG << mImageResizeSize[RIGHT].height << std::endl;
        mMtxImageResize.lock();
        resize(*left, mImageResize[LEFT], mImageResizeSize[LEFT]);
        resize(*right, mImageResize[RIGHT], mImageResizeSize[RIGHT]);

        circle( mImageResize[LEFT],
            Point(mImgMiddleResize[0],mImgMiddleResize[2]),
            5,
            Scalar( 0, 0, 255 ),
            -1);
        circle( mImageResize[RIGHT],
            Point(mImgMiddleResize[1],mImgMiddleResize[3]),
            5,
            Scalar( 0, 0, 255 ),
            -1);
        mMtxImageResize.unlock();
    }

    void OpenVRCompositorListener::setCameraConfig(
        float width, float height,
        float f_x, float f_y,
        float c_x, float c_y,
        int leftOrRightCam)
    {
        mCameraConfig[leftOrRightCam].width = width;
        mCameraConfig[leftOrRightCam].height = height;
        mCameraConfig[leftOrRightCam].f_x = f_x;
        mCameraConfig[leftOrRightCam].f_y = f_y;
        mCameraConfig[leftOrRightCam].c_x = c_x;
        mCameraConfig[leftOrRightCam].c_y = c_y;
        calcAlign();
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameEnded( const Ogre::FrameEvent& evt )
    {
        if(mHMD)
        {
            mVrCompositor3D->PostPresentHandoff();
        }
        return true;
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::workspacePreUpdate( Ogre::CompositorWorkspace *workspace )
    {
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passPreExecute( Ogre::CompositorPass *pass )
    {
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passSceneAfterShadowMaps( Ogre::CompositorPassScene *pass )
    {
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passSceneAfterFrustumCulling( Ogre::CompositorPassScene *pass )
    {
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::setWaitingMode( VrWaitingMode::VrWaitingMode waitingMode )
    {
        mWaitingMode = waitingMode;
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::setGlitchFree( VrWaitingMode::VrWaitingMode firstGlitchFreeMode )
    {
        mFirstGlitchFreeMode = firstGlitchFreeMode;
    }
}
