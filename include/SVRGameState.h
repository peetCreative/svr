#ifndef _SVRGameState_H_
#define _SVRGameState_H_

#include "OgrePrerequisites.h"
#include "OgreCommon/GameState.h"
#include "OgreCommon/CameraController.h"
#include "SVRGraphicsSystem.h"
namespace Ogre
{
        namespace v1
    {
        class TextAreaOverlayElement;
    }
}

namespace Demo
{
    /// Base game state for the tutorials. All it does is show a little text on screen :)
    class SVRGameState : public Demo::GameState
    {
    protected:
        SVRGraphicsSystem      *mGraphicsSystem;
        Ogre::SceneNode     *mLightNodes;
        Ogre::SceneNode     *mSceneNode;
        /// Optional, for controlling the camera with WASD and the mouse
        CameraController    *mCameraController;

        Ogre::String        mHelpDescription;
        Ogre::uint16        mDisplayHelpMode;
        Ogre::uint16        mNumDisplayHelpModes;

        Ogre::v1::TextAreaOverlayElement *mDebugText;
        Ogre::v1::TextAreaOverlayElement *mDebugTextShadow;

        virtual void createDebugTextOverlay(void);
        virtual void generateDebugText( float timeSinceLast, Ogre::String &outText );

        Ogre::Item      *mHiddenAreaMeshVr;
        Ogre::Item      *mCube;
        Ogre::uint8     mTransparencyMode;
        float           mTransparencyValue;

    public:
        SVRGameState( const Ogre::String &helpDescription );
        virtual ~SVRGameState();

        void _notifyGraphicsSystem(SVRGraphicsSystem *graphicsSystem );

        virtual void createScene01(void);

        virtual void update( float timeSinceLast );
        void setTransparencyToMaterials(void);

        virtual void keyPressed( const SDL_KeyboardEvent &arg );
        virtual void keyReleased( const SDL_KeyboardEvent &arg );

        virtual void mouseMoved( const SDL_Event &arg );
    };
}

#endif
