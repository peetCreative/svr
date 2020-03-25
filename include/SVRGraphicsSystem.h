#ifndef _SVRGraphicsSystem_H_
#define _SVRGraphicsSystem_H_

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

#include "OgreCommon/BaseSystem.h"

#include "OgreCommon/System/StaticPluginLoader.h"
#include "OgrePrerequisites.h"
#include "OgreColourValue.h"
#include "OgreOverlayPrerequisites.h"

#include "Threading/OgreUniformScalableTask.h"
#include "OgreCommon/SdlEmulationLayer.h"
#include "OgreOverlaySystem.h"

#include "OpenVRCompositorListener.h"

#if OGRE_USE_SDL2
    #include <SDL.h>
#endif

namespace Demo
{
    class SdlInputHandler;

    class SVRGraphicsSystem : public BaseSystem
    {
        BaseSystem          *mLogicSystem;

        
#if OGRE_USE_SDL2
        SDL_Window          *mSdlWindow;
        SdlInputHandler     *mInputHandler;
#endif
        Ogre::Root                  *mRoot;
        Ogre::Window                *mRenderWindow;
        Ogre::SceneManager          *mSceneManager;
        Ogre::Camera                *mCamera;
        Ogre::CompositorWorkspace   *mVrWorkspace;

        Ogre::String                mPluginsFolder;
        Ogre::String                mWriteAccessFolder;
        Ogre::String                mResourcePath;

        Ogre::v1::OverlaySystem     *mOverlaySystem;

        StaticPluginLoader          mStaticPluginLoader;

        float               mAccumTimeSinceLastLogicFrame;
//        Ogre::uint32        mCurrentTransformIdx;

        bool                mQuit;
        bool                mAlwaysAskForConfig;

        Ogre::ColourValue   mBackgroundColour;

        vr::IVRSystem *mHMD;
        std::string mStrDriver;
        std::string mStrDisplay;
        std::string mDeviceModelNumber;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

        Ogre::TextureGpu            *mVrTexture;
        Ogre::Camera                *mVrCullCamera;

        Demo::OpenVRCompositorListener    *mOvrCompositorListener;

        //Graphics System
        bool isWriteAccessFolder( const Ogre::String &folderPath, const Ogre::String &fileToSave );

#if OGRE_USE_SDL2
        void handleWindowEvent( const SDL_Event& evt );
#endif
        /// @see MessageQueueSystem::processIncomingMessage
        // Keep this, for when we reintroduce a logic frame
        virtual void processIncomingMessage(
            Mq::MessageId messageId, const void *data );

        static void addResourceLocation(
            const Ogre::String &archName,
            const Ogre::String &typeName,
            const Ogre::String &secName );

        virtual void setupResources(void);
        virtual void loadResources(void);

        void registerHlms(void);

        virtual void chooseSceneManager(void);
        virtual void createCamera(void);

        virtual void initMiscParamsListener( Ogre::NameValuePairList &params );

        /// Optional override method where you can create resource listeners (e.g. for loading screens)
        virtual void createResourceListener(void) {}


        // SVRGraphicsSystem
        virtual Ogre::CompositorWorkspace* setupCompositor();


        static std::string GetTrackedDeviceString(
            vr::TrackedDeviceIndex_t unDevice,
            vr::TrackedDeviceProperty prop,
            vr::TrackedPropertyError *peError = NULL );
        void initOpenVR(void);
        void initCompositorVR(void);
        void createHiddenAreaMeshVR(void);

    public:
        SVRGraphicsSystem(
            GameState *gameState,
            bool askForConfig,
            Ogre::ColourValue backgroundColour = Ogre::ColourValue( 0.2f, 0.4f, 0.6f )
        );
        ~SVRGraphicsSystem();

        void _notifyLogicSystem( BaseSystem *logicSystem )      { mLogicSystem = logicSystem; }

        void initialize( const Ogre::String &windowTitle );
        void deinitialize(void);

        void setImgPtr();
        void update( float timeSinceLast );

#if OGRE_USE_SDL2
        SdlInputHandler* getInputHandler(void) { return mInputHandler; }
#endif

        void setQuit(void)                                      { mQuit = true; }
        bool getQuit(void) const                                { return mQuit; }

        float getAccumTimeSinceLastLogicFrame(void) const       { return mAccumTimeSinceLastLogicFrame; }

        Ogre::Root* getRoot(void) const                         { return mRoot; }
        Ogre::Window* getRenderWindow(void) const               { return mRenderWindow; }
        Ogre::SceneManager* getSceneManager(void) const         { return mSceneManager; }
        Ogre::Camera* getCamera(void) const                     { return mCamera; }
        Ogre::CompositorWorkspace* getCompositorWorkspace(void) const { return mVrWorkspace; }
        Ogre::v1::OverlaySystem* getOverlaySystem(void) const   { return mOverlaySystem; }

        const Ogre::String& getPluginsFolder(void) const        { return mPluginsFolder; }
        const Ogre::String& getWriteAccessFolder(void) const    { return mWriteAccessFolder; }
        const Ogre::String& getResourcePath(void) const         { return mResourcePath; }


         OpenVRCompositorListener* getOvrCompositorListener(void) { return mOvrCompositorListener; }

        virtual void stopCompositor(void);
        virtual void restartCompositor(void);

    };
};

#endif
