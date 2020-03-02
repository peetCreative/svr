
#include "SVRGameState.h"
#include "OpenVRCompositorListener.h"

#include "OgreCommon/CameraController.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"

#include "OgreMeshManager.h"
#include "OgreMeshManager2.h"
#include "OgreMesh2.h"

#include "OgreCamera.h"
#include "OgreWindow.h"

#include "OgreHlmsPbsDatablock.h"
#include "OgreHlmsSamplerblock.h"

#include "OgreRoot.h"
#include "OgreHlmsManager.h"
#include "OgreHlmsPbs.h"

#include "OgreOverlayManager.h"
#include "OgreOverlay.h"
#include "OgreOverlayContainer.h"
#include "OgreTextAreaOverlayElement.h"

#include "OgreFrameStats.h"

#include "OgreHlmsManager.h"
#include "OgreHlms.h"
#include "OgreHlmsCompute.h"

#include "OgreGpuProgramManager.h"

namespace Demo
{
    //-----------------------------------------------------------------------------------
	SVRGameState::SVRGameState( const Ogre::String &helpDescription ) :
        mGraphicsSystem( 0 ),
        mCameraController( 0 ),
        mHelpDescription( helpDescription ),
        mDisplayHelpMode( 1 ),
        mNumDisplayHelpModes( 2 ),
        mDebugText( 0 )
	{
    }
//-----------------------------------------------------------------------------------
    SVRGameState::~SVRGameState()
    {
        delete mCameraController;
        mCameraController = 0;
    }

    
	void SVRGameState::createScene01(void)
    {
//         mCameraController = new CameraController( mGraphicsSystem, false );

        createDebugTextOverlay();
    }

	void SVRGameState::_notifyGraphicsSystem(SVRGraphicsSystem *graphicsSystem )
    {
		mGraphicsSystem = graphicsSystem;
    }
    

	    //-----------------------------------------------------------------------------------
    void SVRGameState::createDebugTextOverlay(void)
    {
        Ogre::v1::OverlayManager &overlayManager = Ogre::v1::OverlayManager::getSingleton();
        Ogre::v1::Overlay *overlay = overlayManager.create( "DebugText" );

        Ogre::v1::OverlayContainer *panel = static_cast<Ogre::v1::OverlayContainer*>(
            overlayManager.createOverlayElement("Panel", "DebugPanel"));
        mDebugText = static_cast<Ogre::v1::TextAreaOverlayElement*>(
                    overlayManager.createOverlayElement( "TextArea", "DebugText" ) );
        mDebugText->setFontName( "DebugFont" );
        mDebugText->setCharHeight( 0.025f );

        mDebugTextShadow= static_cast<Ogre::v1::TextAreaOverlayElement*>(
                    overlayManager.createOverlayElement( "TextArea", "0DebugTextShadow" ) );
        mDebugTextShadow->setFontName( "DebugFont" );
        mDebugTextShadow->setCharHeight( 0.025f );
        mDebugTextShadow->setColour( Ogre::ColourValue::Black );
        mDebugTextShadow->setPosition( 0.002f, 0.002f );

        panel->addChild( mDebugTextShadow );
        panel->addChild( mDebugText );
        overlay->add2D( panel );
        overlay->show();
    }
    //-----------------------------------------------------------------------------------
    void SVRGameState::generateDebugText( float timeSinceLast, Ogre::String &outText )
    {
        if( mDisplayHelpMode == 0 )
        {
            outText = mHelpDescription;
            outText += "\n\nPress F1 to toggle help";
            outText += "\n\nProtip: Ctrl+F1 will reload PBS shaders (for real time template editing).\n"
                       "Ctrl+F2 reloads Unlit shaders.\n"
                       "Ctrl+F3 reloads Compute shaders.\n"
                       "Note: If the modified templates produce invalid shader code, "
                       "crashes or exceptions can happen.\n";
            return;
        }

        const Ogre::FrameStats *frameStats = mGraphicsSystem->getRoot()->getFrameStats();

        Ogre::String finalText;
        finalText.reserve( 128 );
        finalText  = "Frame time:\t";
        finalText += Ogre::StringConverter::toString( timeSinceLast * 1000.0f );
        finalText += " ms\n";
        finalText += "Frame FPS:\t";
        finalText += Ogre::StringConverter::toString( 1.0f / timeSinceLast );
        finalText += "\nAvg time:\t";
        finalText += Ogre::StringConverter::toString( frameStats->getAvgTime() );
        finalText += " ms\n";
        finalText += "Avg FPS:\t";
        finalText += Ogre::StringConverter::toString( 1000.0f / frameStats->getAvgTime() );
        finalText += "\n\nPress F1 to toggle help";

        outText.swap( finalText );

        mDebugText->setCaption( finalText );
        mDebugTextShadow->setCaption( finalText );
    }
    //-----------------------------------------------------------------------------------
    void SVRGameState::update( float timeSinceLast )
    {
        if( mDisplayHelpMode != 0 )
        {
            //Show FPS
            Ogre::String finalText;
            generateDebugText( timeSinceLast, finalText );
            mDebugText->setCaption( finalText );
            mDebugTextShadow->setCaption( finalText );
        }

        if( mCameraController )
            mCameraController->update( timeSinceLast );
    }
	
    //-----------------------------------------------------------------------------------
        void SVRGameState::keyPressed( const SDL_KeyboardEvent &arg )
    {
        bool handledEvent = false;

        if( mCameraController )
            handledEvent = mCameraController->keyPressed( arg );

        if( !handledEvent )
            GameState::keyPressed( arg );
    }
    //-----------------------------------------------------------------------------------
    void SVRGameState::keyReleased( const SDL_KeyboardEvent &arg )
    {
        if( arg.keysym.scancode == SDL_SCANCODE_F1 && (arg.keysym.mod & ~(KMOD_NUM|KMOD_CAPS)) == 0 )
        {
            mDisplayHelpMode = (mDisplayHelpMode + 1) % mNumDisplayHelpModes;

            Ogre::String finalText;
            generateDebugText( 0, finalText );
            mDebugText->setCaption( finalText );
            mDebugTextShadow->setCaption( finalText );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F1 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of PBS shaders. We need to clear the microcode cache
            //to prevent using old compiled versions.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getHlms( Ogre::HLMS_PBS );
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F2  && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of Unlit shaders.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getHlms( Ogre::HLMS_UNLIT );
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F3 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of Compute shaders.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getComputeHlms();
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_ESCAPE)
        {
            mGraphicsSystem->setQuit();
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_N)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            ovrListener->toggleNextPict();
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_M)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            ovrListener->triggerWriteTexture();
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_V)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            float imgScale = ovrListener->getImgScale();
            ovrListener->setImgScale(imgScale - 0.05);
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_B)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            float imgScale = ovrListener->getImgScale();
            ovrListener->setImgScale(imgScale + 0.05);
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_X)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            ovrListener->setInputType(
                OpenVRCompositorListener::VIDEO);
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_Y)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            ovrListener->setInputType(
                OpenVRCompositorListener::IMG_SERIES);
        }
        else
        {
            bool handledEvent = false;

            if( mCameraController )
                handledEvent = mCameraController->keyReleased( arg );

            if( !handledEvent )
                GameState::keyReleased( arg );
        }
    }
    //-----------------------------------------------------------------------------------
    void SVRGameState::mouseMoved( const SDL_Event &arg )
    {
        if( mCameraController )
            mCameraController->mouseMoved( arg );

        GameState::mouseMoved( arg );
    }

}
