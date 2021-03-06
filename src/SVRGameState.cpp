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

#include "OgreMeshManager.h"
#include "OgreMeshManager2.h"
#include "OgreMesh2.h"

#include "OgreHlmsManager.h"
#include "OgreHlms.h"
#include "OgreHlmsCompute.h"

#include "OgreGpuProgramManager.h"

#include <OgreRectangle2D2.h>

namespace Demo
{
    //-----------------------------------------------------------------------------------
	SVRGameState::SVRGameState( const Ogre::String &helpDescription ) :
        mGraphicsSystem( 0 ),
        mCameraController( 0 ),
        mHelpDescription( helpDescription ),
        mDisplayHelpMode( 1 ),
        mNumDisplayHelpModes( 2 ),
        mDebugText( 0 ),
        mCube( nullptr ),
        mTransparencyMode( 0 ),
        mTransparencyValue( 0.5 )
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
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
        const bool bIsHamVrOptEnabled = 
            !Ogre::MeshManager::getSingleton().getByName(
                "HiddenAreaMeshVr.mesh",
                Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME
            ).isNull();

        if( bIsHamVrOptEnabled )
        {
            mHiddenAreaMeshVr =
                sceneManager->createItem(
                    "HiddenAreaMeshVr.mesh",
                    Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME,
                    Ogre::SCENE_STATIC );
            mHiddenAreaMeshVr->setCastShadows( false );
            mHiddenAreaMeshVr->setRenderQueueGroup( 0u );
            mHiddenAreaMeshVr->getSubItem(0)->setUseIdentityProjection( true );
            // Set to render *after* the RadialDensityMask
            mHiddenAreaMeshVr->getSubItem(0)->setRenderQueueSubGroup( 1u );
            sceneManager->getRootSceneNode( Ogre::SCENE_STATIC )->attachObject( mHiddenAreaMeshVr );
        }

        mCube = sceneManager->createItem(
            "Cube_d.mesh",
            Ogre::ResourceGroupManager::
            AUTODETECT_RESOURCE_GROUP_NAME,
            Ogre::SCENE_DYNAMIC );

        mCube->setVisibilityFlags( 0x000000002 );
        mSceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );

        mSceneNode->setPosition( 0, 0, 0 );

        mSceneNode->attachObject( mCube );

        Ogre::SceneNode *rootNode = sceneManager->getRootSceneNode();

        Ogre::Light *light = sceneManager->createLight();
        Ogre::SceneNode *lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setPowerScale( 1.0f );
        light->setType( Ogre::Light::LT_DIRECTIONAL );
        light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

        mLightNodes = lightNode;
        sceneManager->setAmbientLight(
            Ogre::ColourValue( 0.3f, 0.5f, 0.7f ) * 0.1f * 0.75f,
            Ogre::ColourValue( 0.6f, 0.45f, 0.3f ) * 0.065f * 0.75f,
            -light->getDirection() + Ogre::Vector3::UNIT_Y * 0.2f );
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

        mDebugTextShadow = static_cast<Ogre::v1::TextAreaOverlayElement*>(
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
    void SVRGameState::setTransparencyToMaterials(void)
    {
        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->getRoot()->getHlmsManager();

        assert( dynamic_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms( Ogre::HLMS_PBS ) ) );

        Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

        Ogre::HlmsPbsDatablock::TransparencyModes mode =
                static_cast<Ogre::HlmsPbsDatablock::TransparencyModes>( mTransparencyMode );

        if( mTransparencyValue >= 1.0f )
            mode = Ogre::HlmsPbsDatablock::None;

        if( mTransparencyMode < 1.0f && mode == Ogre::HlmsPbsDatablock::None )
            mode = Ogre::HlmsPbsDatablock::Transparent;

        Ogre::HlmsPbsDatablock *datablock =
            static_cast<Ogre::HlmsPbsDatablock*>(
                hlmsPbs->getDatablock( "Cube" ) );

        datablock->setTransparency( mTransparencyValue, mode );
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
        else if( arg.keysym.sym == SDLK_F4 )
        {
            Ogre::uint32 visibilityMask = mGraphicsSystem->getSceneManager()->getVisibilityMask();
            bool showPalette = (visibilityMask & 0x00000002) != 0;
            showPalette = !showPalette;
            visibilityMask &= ~0x00000002;
            visibilityMask |= (Ogre::uint32)(showPalette) << 1;
            mGraphicsSystem->getSceneManager()->setVisibilityMask( visibilityMask );
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_ESCAPE)
        {
            mGraphicsSystem->setQuit();
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_N)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            ovrListener->showMovie();
        }
        else if(arg.keysym.scancode == SDL_SCANCODE_M)
        {
            OpenVRCompositorListener *ovrListener = mGraphicsSystem->getOvrCompositorListener();
            ovrListener->triggerWriteTexture();
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_KP_PLUS )
        {
            if( mTransparencyValue < 1.0f )
            {
                mTransparencyValue += 0.1f;
                mTransparencyValue = Ogre::min( mTransparencyValue, 1.0f );
                setTransparencyToMaterials();
            }
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_MINUS ||
                 arg.keysym.scancode == SDL_SCANCODE_KP_MINUS )
        {
            if( mTransparencyValue > 0.0f )
            {
                mTransparencyValue -= 0.1f;
                mTransparencyValue = Ogre::max( mTransparencyValue, 0.0f );
                setTransparencyToMaterials();
            }
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
