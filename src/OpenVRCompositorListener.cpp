
#include "OpenVRCompositorListener.h"
#include "SVR.h"

#include "OgreTextureGpuManager.h"
#include "OgrePixelFormatGpuUtils.h"
#include "OgreMemoryAllocatorConfig.h"
#include "OgreGpuResource.h"
#include "OgreStagingTexture.h"
#include "OgreLogManager.h"

#include "opencv2/opencv.hpp"

#include "OgreTextureGpu.h"
#include "OgreRenderSystem.h"
#include "Compositor/Pass/OgreCompositorPass.h"
#include "Compositor/Pass/OgreCompositorPassDef.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassScene.h"

#include "Compositor/OgreCompositorWorkspace.h"

#include <sstream>
#include <cmath>
#include <experimental/filesystem>
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
            Ogre::Camera *camera, Ogre::Camera *cullCamera ) :
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
        mImgScale(0.3),
        mImgRatio(640/512.0),
        mImageResizeSize{Size(0,0), Size(0,0)},
        mImageOrig{nullptr, nullptr},
        mImageCnt(0)
    {
        memset( mTrackedDevicePose, 0, sizeof( mTrackedDevicePose ) );
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

        mInputType = CONST_MAT;
        if(mInputType == VIDEO)
            initVideoInput();
        else if (mInputType == IMG_TIMESTAMP)
            initImgsTimestamp();


        if(mHMD)
        {
            mCamera->setVrData( &mVrData );
            syncCameraProjection( true );
    //         calcAlign();

            std::cout << "camera left to right " << mVrData.mLeftToRight.x <<
                " "  << mVrData.mLeftToRight.y  << std::endl;
            float left, right, top, bottom;
            mHMD->GetProjectionRaw(vr::Eye_Left, &left, &right, &top, &bottom);
            std::cout << "left camera tan left and right " << left << " " << right << std::endl;
            std::cout << "left camera tan top and bottom " << top << " " << bottom << std::endl;
            mHMD->GetProjectionRaw(vr::Eye_Right, &left, &right, &top, &bottom);
            std::cout << "right camera tan left and right " << left << " " << right << std::endl;
            std::cout << "right camera tan top and bottom " << top << " " << bottom << std::endl;
            mRoot->addFrameListener( this );
            mWorkspace->setListener( this );
        }

        const Ogre::String &renderSystemName = mRenderSystem->getName();
        if( renderSystemName == "OpenGL 3+ Rendering Subsystem" )
            mApiTextureType = vr::TextureType_OpenGL;
        mWriteTexture = true;


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
        const size_t bytesPerRow =
            mVrTexture->_getSysRamCopyBytesPerRow( 0 );

        mImageData = reinterpret_cast<uint8*>(
            OGRE_MALLOC_SIMD( dataSize,
            MEMCATEGORY_RESOURCE ) );
        memset(mImageData, 25, dataSize);

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

    bool OpenVRCompositorListener::initVideoInput() {
            mCapture.set(CV_CAP_PROP_MODE,  CV_CAP_MODE_RGB );
            mCapture = VideoCapture("/home/peetcreative/SurgicalData/ForPeter/Video.avi");
            if (!mCapture.isOpened()) {
                std::cout << "Video could not be opened" << std::endl;
                return false;
            }
            // Default resolution of the frame is obtained.The default resolution is system dependent.
            mCaptureFrameWidth =
                mCapture.get(CV_CAP_PROP_FRAME_WIDTH);
            mCaptureFrameHeight =
                mCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
            mCaptureFramePixelFormat =
                mCapture.get(CV_CAP_PROP_FORMAT);
            return true;
    }

    void OpenVRCompositorListener::initImgsTimestamp() {
        mFileIteratorLeft = fs::directory_iterator("/tmp/new_images");
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
    void OpenVRCompositorListener::updateHmdTrackingPose(void)
    {
        mVrCompositor3D->WaitGetPoses( mTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
/*
        mValidPoseCount = 0;
        for( size_t nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
        {
            if ( mTrackedDevicePose[nDevice].bPoseIsValid )
            {
                ++mValidPoseCount;
                mDevicePose[nDevice] = convertSteamVRMatrixToMatrix4(
                                           mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
            }
        }

        if( mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
        {
            const bool canSync = canSyncCameraTransformImmediately();
            if( canSync )
                syncCamera();
            else
                mMustSyncAtEndOfFrame = true;
        }*/
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::syncCullCamera(void)
    {
        const Ogre::Quaternion derivedRot = mCamera->getDerivedOrientation();
        Ogre::Vector3 camPos = mCamera->getDerivedPosition();
        mVrCullCamera->setOrientation( derivedRot );
        mVrCullCamera->setPosition( camPos + derivedRot * mCullCameraOffset );
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::syncCamera(void)
    {
        OGRE_ASSERT_MEDIUM( mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid );
        mCamera->setPosition( mDevicePose[vr::k_unTrackedDeviceIndex_Hmd].getTrans() );
        mCamera->setOrientation( mDevicePose[vr::k_unTrackedDeviceIndex_Hmd].extractQuaternion() );

        if( mWaitingMode < VrWaitingMode::AfterFrustumCulling )
            syncCullCamera();

        mMustSyncAtEndOfFrame = false;
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
                mHMD->GetProjectionRaw( eyeIdx, &eyeFrustumExtents[i].x, &eyeFrustumExtents[i].y,
                                        &eyeFrustumExtents[i].z, &eyeFrustumExtents[i].w );
            }

            mVrData.set( eyeToHead, projectionMatrixRS );
            mLastCamNear = camNear;
            mLastCamFar = camFar;

            Ogre::Vector4 cameraCullFrustumExtents;
            cameraCullFrustumExtents.x = std::min( eyeFrustumExtents[0].x, eyeFrustumExtents[1].x );
            cameraCullFrustumExtents.y = std::max( eyeFrustumExtents[0].y, eyeFrustumExtents[1].y );
            cameraCullFrustumExtents.z = std::max( eyeFrustumExtents[0].z, eyeFrustumExtents[1].z );
            cameraCullFrustumExtents.w = std::min( eyeFrustumExtents[0].w, eyeFrustumExtents[1].w );

            mVrCullCamera->setFrustumExtents( cameraCullFrustumExtents.x, cameraCullFrustumExtents.y,
                                              cameraCullFrustumExtents.w, cameraCullFrustumExtents.z,
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
        if(mImgScale <= 0)
        {
            return false;
        }

        if (mInputType == VIDEO)
        {
            mImgRatio =  float(mCaptureFrameWidth)/mCaptureFrameHeight;
        }
        else if( mInputType == IMG_SERIES )
        {
            mImgRatio =  640/512.0;
        }


        //now we have to know
        size_t vr_width_half = mVrTexture->getWidth()/2;

//         float img_height_resize_left = mVrTexture->getHeight() * mImgScale;
//         float img_width_resize_right = img_height_resize * mImgRatio;

        //left_left, left_right
        //right_left right_right
        //left_top left_bottom
        //right_top right_bottom
        float tan[4][2];

        //leftLeft, rightLeft leftTop rightTop
        size_t align[4];
        float c_vr[4];
        float  img_size_resize[4];
        float  img_middle_resize[4];
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

        mHMD->GetProjectionRaw(
            vr::Eye_Left, &tan[0][0], &tan[0][1], &tan[2][0], &tan[2][1]);
        mHMD->GetProjectionRaw(
            vr::Eye_Right, &tan[1][0], &tan[1][1], &tan[3][0], &tan[3][1]);

        std::cout << "ratio left: " << vr_width_half << " " << mVrTexture->getHeight() << std::endl;
        std::cout << "ratio left: " << -tan[0][0] + tan[0][1] << " " <<
            -tan[2][0] + tan[2][1] << std::endl;
        std::cout << "ratio right: " << -tan[0][0] + tan[0][1] << " " <<
            -tan[2][0] + tan[2][1] << std::endl;

        for (int i = 0; i < 4; i++)
        {
            c_vr[i] = -tan[i][0] * vr_size[i] / (-tan[i][0] + tan[i][1]);
            img_size_resize[i] = c_vr[i] * img_size[i] / (f_cam[i] * -tan[i][0]);
            img_middle_resize[i] = img_size_resize[i] * c_cam[i] / img_size[i]; 
            float align_f = c_vr[i] - img_middle_resize[i];
            if (align_f <= 0)
                return false;
            align[i] = static_cast<size_t>(std::round(align_f));
            OGRE_ASSERT_MSG(img_size_resize[i] > 0, "img_height_resize smaller than zero");
            if (i < 2) {
                mImageResizeSize[i%2].width =
                    static_cast<size_t>(std::round(img_size_resize[i]));
            } else {
                mImageResizeSize[i%2].height =
                    static_cast<size_t>(std::round(img_size_resize[i]));
            }
        }

        mImageResize[LEFT] = Mat();
        mImageResize[RIGHT] = Mat();
        if (mInputType == CONST_MAT)
        {
            mImageOrig[LEFT] = nullptr;
            mImageOrig[RIGHT] = nullptr;
        }
        else if ( mInputType == VIDEO)
        {
            Size origSize(1920, 540);
            mImageOrig[LEFT] = new Mat(origSize, CV_8UC3);
            mImageOrig[RIGHT] = new Mat(origSize, CV_8UC3);
        }
        else
        {
            Size origSizeLeft(
                mCameraConfig[LEFT].width, mCameraConfig[LEFT].height);
            Size origSizeRight(
                mCameraConfig[RIGHT].width, mCameraConfig[RIGHT].height);
            mImageOrig[LEFT] = new Mat(origSizeLeft, CV_8UC3);
            mImageOrig[RIGHT] = new Mat(origSizeRight, CV_8UC3);
        }

        std::cout <<"left resize:" << mImageResizeSize[LEFT].width 
            << " " << mImageResizeSize[LEFT].height << std::endl;
        std::cout <<"right resize:" << mImageResizeSize[RIGHT].width 
            << " " << mImageResizeSize[RIGHT].height << std::endl;
        mAlign.leftLeft = align[0];
        mAlign.rightLeft = vr_width_half + align[1];
        mAlign.leftTop = align[2];
        mAlign.rightTop = align[3];

        return true;

//         std::cout << "Align leftLeft:" << mAlign.leftLeft << std::endl;
//         std::cout << "Align leftTop:" << mAlign.leftTop << std::endl;
//         std::cout << "Align rightLeft:" << mAlign.rightLeft << std::endl;
//         std::cout << "Align rightTop:" << mAlign.rightTop << std::endl;
    }
    bool OpenVRCompositorListener::fillTexture(void)
    {
        const size_t bytesPerPixel = 4u;
        const size_t bytesPerRow =
            mVrTexture->_getSysRamCopyBytesPerRow( 0 );

        if(mInputType != CONST_MAT) {
            if(mInputType == VIDEO)
            {
                cv::Mat mMat;

                // Capture frame-by-frame
                mCapture >> mMat; //1920/1080
                //std::cout <<"type:" << std::endl;
                //std::cout <<"type:" << mCapture.get(CV_CAP_PROP_FORMAT ) << std::endl;
                //<< " width:" << mCapture.get(CV_CAP_PROP_FRAME_WIDTH)
                //<< " height:" << mCapture.get(CV_CAP_PROP_FRAME_HEIGHT)
                //<< std::endl;
                //1920x540
                if(mMat.empty())
                {
                    return false;
                }
                cv::Rect lrect(0,540, 1920, 540);
                cv::Rect rrect(0,0, 1920, 540);
                *mImageOrig[LEFT] = mMat(lrect);
                *mImageOrig[RIGHT] = mMat(rrect);
            }
            if (mInputType == IMG_SERIES)
            {
                if (mImageCnt >= 1000)
                    mImageCnt = 0;
                std::stringstream ssl;
                ssl << "/tmp/rect/left_undist_rect01"
                    << std::setfill('0') << std::setw(3) << mImageCnt
                    << ".png";
                std::stringstream ssr;
                ssr << "/tmp/rect/right_undist_rect01"
                    << std::setfill('0') << std::setw(3) << mImageCnt
                    << ".png";
        //         std::cout << "fill texture with "<< ssl.str() << std::endl;
        //         std::cout << "fill texture with "<< ssr.str() << std::endl;
    //             mImageCnt++;
                *mImageOrig[LEFT] = imread(ssl.str());
                *mImageOrig[RIGHT] = imread(ssr.str());
            }
            if (mInputType == IMG_TIMESTAMP)
            {
                fs::directory_entry entry_left = *mFileIteratorLeft++;
                std::string path_str_left = entry_left.path().string();
                size_t pos_left = path_str_left.rfind("left");
                std::string path_str_right = path_str_left.substr(0, pos_left) +
                    "right" +
                    path_str_left.substr(pos_left + 4, path_str_left.length());
                fs::path path_right(path_str_right);
                if(!exists(path_right))
                    return false;
                *mImageOrig[LEFT] = imread(path_str_right);
                *mImageOrig[RIGHT] = imread(path_str_left);
            }

            if(mImageOrig[LEFT]->empty() || mImageOrig[RIGHT]->empty())
            {
                return false;
            }

            resize(*mImageOrig[LEFT], mImageResize[LEFT], mImageResizeSize[LEFT]);
            resize(*mImageOrig[RIGHT], mImageResize[RIGHT], mImageResizeSize[RIGHT]);
        }

//         std::cout << mVrTexture->getWidth() << std::endl;
//         std::cout << mVrTexture->getHeight() << std::endl;
//         std::cout << width_resize << std::endl;
//         std::cout << "height_resize1 " << height_resize << std::endl;
//         std::cout << height_resize << std::endl;

//         std::cout << "width_resize " << width_resize << std::endl;
//         std::cout << "height_resize " << height_resize << std::endl;
//         std::cout << "ldst.cols " << ldst.cols << std::endl;
//         std::cout << "ldst.rows " << ldst.rows << std::endl;

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
    bool OpenVRCompositorListener::frameStarted( const Ogre::FrameEvent& evt )
    {

        if( mWaitingMode == VrWaitingMode::BeforeSceneGraph )
            updateHmdTrackingPose();

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
        if (mWriteTexture) {
            mVrTexture->writeContentsToFile("texture.bmp", 8, 255);
            mWriteTexture = false;
        }
        TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();
        textureManager->waitForStreamingCompletion();

        mVrTexture->waitForData();
        mVrTexture->getCustomAttribute( Ogre::TextureGpu::msFinalTextureBuffer, &eyeTexture.handle );

        texBounds.uMin = 0;
        texBounds.uMax = 0.5f;
        mVrCompositor3D->Submit( vr::Eye_Left, &eyeTexture, &texBounds );
        texBounds.uMin = 0.5f;
        texBounds.uMax = 1.0f;
        mVrCompositor3D->Submit( vr::Eye_Right, &eyeTexture, &texBounds );

        mRenderSystem->flushCommands();


//         vr::VREvent_t event;
//         while( mHMD->PollNextEvent( &event, sizeof(event) ) )
//         {
//             if( event.trackedDeviceIndex != vr::k_unTrackedDeviceIndex_Hmd &&
//                 event.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid )
//             {
//                 continue;
//             }
//
//             switch( event.eventType )
//             {
//             case vr::VREvent_TrackedDeviceUpdated:
//             case vr::VREvent_IpdChanged:
//             case vr::VREvent_ChaperoneDataHasChanged:
//                 syncCameraProjection( true );
//                 break;
//             }
//         }

        return true;
    }

    float OpenVRCompositorListener::getImgScale()
    {
        return mImgScale;
    }

    void OpenVRCompositorListener::setImgScale(float imgScale)
    {
        float oldImgScale = mImgScale;
        mImgScale = imgScale;
        if (!calcAlign()) {
            mImgScale = oldImgScale;
            calcAlign();
        }
    }

    void OpenVRCompositorListener::setImgPtr(const cv::Mat *left, const cv::Mat *right)
    {
        mMtxImageResize.lock();
        resize(*left, mImageResize[LEFT], mImageResizeSize[LEFT]);
        resize(*right, mImageResize[RIGHT], mImageResizeSize[RIGHT]);
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

    InputType OpenVRCompositorListener::getInputType()
    {
        return mInputType;
    }

    void OpenVRCompositorListener::setInputType(
        InputType type)
    {
        mInputType = type;
        if (mInputType == VIDEO)
        {
            initVideoInput();
        }
        calcAlign();
        std::cout << "after InputType set" << std::endl;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameEnded( const Ogre::FrameEvent& evt )
    {
        if(mHMD)
        {
            syncCameraProjection( false );
            if( mWaitingMode == VrWaitingMode::AfterSwap )
                updateHmdTrackingPose();
            else
                mVrCompositor3D->PostPresentHandoff();
            if( mMustSyncAtEndOfFrame )
                syncCamera();
            if( mWaitingMode >= VrWaitingMode::AfterFrustumCulling )
                syncCullCamera();
            return true;
        }
        return true;
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::workspacePreUpdate( Ogre::CompositorWorkspace *workspace )
    {
        if( mWaitingMode == VrWaitingMode::AfterSceneGraph )
            updateHmdTrackingPose();
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passPreExecute( Ogre::CompositorPass *pass )
    {
        if( mWaitingMode == VrWaitingMode::BeforeShadowmaps &&
            pass->getDefinition()->getType() == Ogre::PASS_SCENE &&
            pass->getDefinition()->mIdentifier == 0x01234567 )
        {
            updateHmdTrackingPose();
        }
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passSceneAfterShadowMaps( Ogre::CompositorPassScene *pass )
    {
        if( mWaitingMode == VrWaitingMode::BeforeFrustumCulling &&
            pass->getDefinition()->mIdentifier == 0x01234567 )
        {
            updateHmdTrackingPose();
        }
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passSceneAfterFrustumCulling( Ogre::CompositorPassScene *pass )
    {
        if( mWaitingMode == VrWaitingMode::AfterFrustumCulling &&
            pass->getDefinition()->mIdentifier == 0x01234567 )
        {
            updateHmdTrackingPose();
        }
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
    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::canSyncCameraTransformImmediately(void) const
    {
        return mWaitingMode <= VrWaitingMode::BeforeSceneGraph ||
               mWaitingMode <= mFirstGlitchFreeMode;
    }
}
