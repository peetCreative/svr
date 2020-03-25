
#include "OpenVRCompositorListener.h"

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

using namespace Ogre;
using namespace cv;
namespace fs = std::experimental::filesystem;

namespace Demo
{
    OpenVRCompositorListener::OpenVRCompositorListener(
            vr::IVRSystem *hmd, vr::IVRCompositor *vrCompositor3D,
            vr::IVROverlay *vrCompositor2D,
            Ogre::TextureGpu *vrTexture, Ogre::Root *root,
            Ogre::CompositorWorkspace *workspace,
            Ogre::Camera *camera, Ogre::Camera *cullCamera ) :
        mHMD( hmd ),
        mVrCompositor3D( vrCompositor3D ),
        mVrCompositor2D( vrCompositor2D ),
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
        mImgWidthResize{0,0},
        mImgHeightResize{0,0},
        mImageOrig{nullptr, nullptr},
        mMagicCnt(0),
        mImageCnt(0)
    {
        memset( mTrackedDevicePose, 0, sizeof( mTrackedDevicePose ) );
        memset( mDevicePose, 0, sizeof( mDevicePose ) );
        memset( &mVrData, 0, sizeof( mVrData ) );
        mAlign.leftLeft = 0;
        mAlign.leftTop = 0;
        mAlign.rightLeft = 0;
        mAlign.rightTop = 0;

        mInputType = IMG_TIMESTAMP;
        if(mInputType == VIDEO)
            initVideoInput();
        else if (mInputType == IMG_TIMESTAMP)
            initImgsTimestamp();


        mCamera->setVrData( &mVrData );
        syncCameraProjection( true );
        calcAlign();

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

        const Ogre::String &renderSystemName = mRenderSystem->getName();
        if( renderSystemName == "OpenGL 3+ Rendering Subsystem" )
            mApiTextureType = vr::TextureType_OpenGL;
        else if( renderSystemName == "Direct3D11 Rendering Subsystem" )
            mApiTextureType = vr::TextureType_DirectX;
        else if( renderSystemName == "Metal Rendering Subsystem" )
            mApiTextureType = vr::TextureType_Metal;
        mNextPict = true;
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

    void OpenVRCompositorListener::initVideoInput() {
            mCapture.set(CV_CAP_PROP_MODE,  CV_CAP_MODE_RGB );
            mCapture = VideoCapture("/tmp/Video.avi");
            // Default resolution of the frame is obtained.The default resolution is system dependent.
            mCaptureFrameWidth =
                mCapture.get(CV_CAP_PROP_FRAME_WIDTH);
            mCaptureFrameHeight =
                mCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
            mCaptureFramePixelFormat =
                mCapture.get(CV_CAP_PROP_FORMAT);
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

        //TODO: read the correct camera calibration info
        //DUMMY CAMERACALIBRATION READ CORRECT ONE
        float img_size_x = 640;
        float img_size_y = 512;
        float f_x_left = 534.195;
        float f_y_left = 534.095;
        float c_x_left = 300.45;
        float c_y_left = 250.37;
        float f_x_right = 533.915;
        float f_y_right = 533.8;
        float c_x_right = 349.11;
        float c_y_right = 250.825;

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
        float img_size[4] = {img_size_x, img_size_x,
            img_size_y, img_size_y};
        float f_cam[4] = {f_x_left, f_x_right, f_y_left, f_y_right};
        float c_cam[4] = {c_x_left, c_x_right, c_y_left, c_y_right};

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
                mImgWidthResize[i%2] =
                    static_cast<size_t>(std::round(img_size_resize[i]));
            } else {
                mImgHeightResize[i%2] =
                    static_cast<size_t>(std::round(img_size_resize[i]));
            }
        }

        if (mInputType == ROS)
        {
            mImageOrig[0] = nullptr;
            mImageOrig[1] = nullptr;
        }
        else
        {
            Size origSize(img_size_x, img_size_y);
            mImageOrig[0] = new Mat(origSize, CV_8UC3);
            mImageOrig[1] = new Mat(origSize, CV_8UC3);
        }

        std::cout <<"left  resize:" << mImgWidthResize[0] 
            << " " << mImgHeightResize[0] << std::endl;
        std::cout <<"right resize:" << mImgWidthResize[1] 
            << " " << mImgHeightResize[1] << std::endl;
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

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameStarted( const Ogre::FrameEvent& evt )
    {
        if(mNextPict) {
            fillTexture();
            mNextPict = false;
        }

        if( mWaitingMode == VrWaitingMode::BeforeSceneGraph )
            updateHmdTrackingPose();

        return true;
    }

    bool OpenVRCompositorListener::fillTexture(void)
    {
        if (mMagicCnt-- != 0) {
            return false;
        }

        const size_t bytesPerPixel = 4u;
        const size_t bytesPerRow =
            mVrTexture->_getSysRamCopyBytesPerRow( 0 );

//         memset(mImageData, 25, dataSize);
//         if (mInputType == ROS)
//         {
//             mImageOrig[0] = mLeftROSImgPtr;
//             mImageOrig[1] = mRightROSImgPtr;
//         }
        if(mInputType == VIDEO)
        {
            // Capture frame-by-frame
            mCapture >> mMat; //1920/1080
            //std::cout <<"type:" << std::endl;
            //std::cout <<"type:" << mCapture.get(CV_CAP_PROP_FORMAT ) << std::endl;
            //<< " width:" << mCapture.get(CV_CAP_PROP_FRAME_WIDTH)
            //<< " height:" << mCapture.get(CV_CAP_PROP_FRAME_HEIGHT)
            //<< std::endl;
            //1920x540
            cv::Rect lrect(0,540, 1920, 540);
            cv::Rect rrect(0,0, 1920, 540);
            *mImageOrig[0] = mMat(lrect);
            *mImageOrig[1] = mMat(rrect);
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
            *mImageOrig[0] = imread(ssl.str());
            *mImageOrig[1] = imread(ssr.str());
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
            *mImageOrig[0] = imread(path_str_right);
            *mImageOrig[1] = imread(path_str_left);
        }

        if(mImageOrig[0]->empty() || mImageOrig[1]->empty())
        {
            return false;
        }

        Mat ldst = Mat();
        Mat rdst = Mat();
        Size leftSize = Size(mImgWidthResize[0],  mImgHeightResize[0]);
        Size rightSize = Size(mImgWidthResize[1], mImgHeightResize[1]);
        resize(*mImageOrig[0], ldst, leftSize);
        resize(*mImageOrig[1], rdst, rightSize);

//         std::cout << mVrTexture->getWidth() << std::endl;
//         std::cout << mVrTexture->getHeight() << std::endl;
//         std::cout << width_resize << std::endl;
//         std::cout << "height_resize1 " << height_resize << std::endl;
//         std::cout << height_resize << std::endl;

//         std::cout << "width_resize " << width_resize << std::endl;
//         std::cout << "height_resize " << height_resize << std::endl;
//         std::cout << "ldst.cols " << ldst.cols << std::endl;
//         std::cout << "ldst.rows " << ldst.rows << std::endl;

        if ( !ldst.empty() && !rdst.empty()) {
            size_t align_left;
            size_t align_top;
            Mat* dst;
            for(size_t i = 0; i < 2u; i++) {
                if (i == 0) {
                    align_left = mAlign.leftLeft;
                    align_top = mAlign.leftTop;
                    dst = &ldst;
                }
                else {
                    align_left =  mAlign.rightLeft;
                    align_top = mAlign.rightTop;
                    dst = &rdst;
                }
                size_t row_cnt = align_top * bytesPerRow;
                for (size_t y = 0; y < mImgHeightResize[i]; y++) {
                    size_t cnt = row_cnt + (align_left * bytesPerPixel);
                    uint8_t* img_row_ptr = dst->ptr<uint8_t>(y);
                    for (size_t x = 0; x < mImgWidthResize[i]; x++) {
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

        //Call this function to indicate you're going to start calling mapRegion. startMapRegion
        //must be called from main thread.
        mStagingTexture->startMapRegion();
        //Map region of the staging mVrTexture. This function can be called from any thread after
        //startMapRegion has already been called.
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
        //stopMapRegion indicates you're done calling mapRegion. Call this from the main thread.
        //It is your responsability to ensure you're done using all pointers returned from
        //previous mapRegion calls, and that you won't call it again.
        //You cannot upload until you've called this function.
        //Do NOT call startMapRegion again until you're done with upload() calls.
        mStagingTexture->stopMapRegion();
        //Upload an area of the staging texture into the texture. Must be done from main thread.
        //The last bool parameter, 'skipSysRamCopy', is only relevant for AlwaysKeepSystemRamCopy
        //textures, and we set it to true because we know it's already up to date. Otherwise
        //it needs to be false.
        mStagingTexture->upload( texBox, mVrTexture, 0, 0, 0, false );

        //This call is very important. It notifies the texture is fully ready for being displayed.
        //Since we've scheduled the texture to become resident and pp until now, the texture knew
        //it was being loaded and that only the metadata was certain. This call here signifies
        //loading is done; and any registered listeners will be notified.
        mVrTexture->notifyDataIsReady();
        
        mMagicCnt = 1;
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

    void OpenVRCompositorListener::setImgPtr(cv::Mat *left, cv::Mat *right)
    {
        mImageOrig[0] = left;
        mImageOrig[1] = right;
    }


    OpenVRCompositorListener::InputType
        OpenVRCompositorListener::getInputType()
    {
        return mInputType;
    }

    void OpenVRCompositorListener::setInputType(
        OpenVRCompositorListener::InputType type)
    {
        mInputType = type;
        if (mInputType == VIDEO)
        {
            initVideoInput();
        }
        calcAlign();
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameEnded( const Ogre::FrameEvent& evt )
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
