#include "SVRGraphicsSystem.h"

#include "OgreCommon/GameState.h"
#include "OgreCommon/SdlInputHandler.h"

#include "OgreRoot.h"
#include "OgreException.h"
#include "OgreConfigFile.h"

#include "OgreCamera.h"
#include "OgreItem.h"

#include "Compositor/OgreCompositorManager2.h"

#include "OgreOverlaySystem.h"
#include "OgreOverlayManager.h"

#include "OgreTextureGpuManager.h"

#include "OgreWindowEventUtilities.h"
#include "OgreWindow.h"

#include "OgreFileSystemLayer.h"

// #include "OgreHlmsDiskCache.h"
#include "OgreGpuProgramManager.h"

#include "OgreLogManager.h"

//GraphicsSystem
#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
#include "OgreConfigFile.h"
#include "Compositor/OgreCompositorManager2.h"
#include "OgreHiddenAreaMeshVr.h"

//Hlms
#include "OgreHlmsUnlit.h"
#include "OgreHlmsPbs.h"
#include "OgreHlmsManager.h"
#include "OgreArchiveManager.h"


#include "OgreTextureGpuManager.h"

#include <experimental/filesystem>
#include <iostream>

#if OGRE_USE_SDL2
    #include <SDL_syswm.h>
#endif

#define USE_OPEN_VR

namespace Demo {
    SVRGraphicsSystem::SVRGraphicsSystem(
            GameState *gameState,
            bool askForConfig ) :
        BaseSystem( gameState ),
        mLogicSystem( 0 ),
    #if OGRE_USE_SDL2
        mSdlWindow( 0 ),
        mInputHandler( 0 ),
    #endif
        mRoot( 0 ),
        mRenderWindow( 0 ),
        mSceneManager( 0 ),
        mCamera( 0 ),
        mVrWorkspace( 0 ),
        mPluginsFolder( PLUGINS_FOLDER ),
        mOverlaySystem( 0 ),
        mAccumTimeSinceLastLogicFrame( 0 ),
//         mCurrentTransformIdx( 0 ),
        mQuit( false ),
        mAlwaysAskForConfig( askForConfig ),
        mBackgroundColour( Ogre::ColourValue( 0.2f, 0.4f, 0.6f ) ),
        mHMD( 0 ),
        mVrTexture( 0 ),
        mVrCullCamera( 0 ),
        mOvrCompositorListener( 0 )
    {
        if( isWriteAccessFolder( mPluginsFolder, "Ogre.log" ) )
            mWriteAccessFolder = mPluginsFolder;
        else
        {
            Ogre::FileSystemLayer filesystemLayer( OGRE_VERSION_NAME );
            mWriteAccessFolder = filesystemLayer.getWritablePath( "" );
        }
        memset( mTrackedDevicePose, 0, sizeof (mTrackedDevicePose) );
        mResourcePath = RESOURCE_PATH;
    }
    //-----------------------------------------------------------------------------------
    SVRGraphicsSystem::~SVRGraphicsSystem()
    {
        if( mRoot )
        {
            Ogre::LogManager::getSingleton().logMessage(
                        "WARNING: GraphicsSystem::deinitialize() not called!!!", Ogre::LML_CRITICAL );
        }
    }

        //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::initialize(
        InputType inputType, const Ogre::String &windowTitle )
    {
        std::cout << "init graphics" << std::endl;
#if OGRE_USE_SDL2
        //if( SDL_Init( SDL_INIT_EVERYTHING ) != 0 )
        if( SDL_Init( SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK |
                      SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS ) != 0 )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR, "Cannot initialize SDL2!",
                         "GraphicsSystem::initialize" );
        }
#endif

        Ogre::String pluginsPath;
        // only use plugins.cfg if not static
    #ifndef OGRE_STATIC_LIB
    #if OGRE_DEBUG_MODE
        pluginsPath = mPluginsFolder + "plugins_d.cfg";
    #else
        pluginsPath = mPluginsFolder + "plugins.cfg";
    #endif
    #endif

        mRoot = OGRE_NEW Ogre::Root( pluginsPath,
                                     mWriteAccessFolder + "ogre.cfg",
                                     mWriteAccessFolder + "Ogre.log" );

        mStaticPluginLoader.install( mRoot );

        // enable sRGB Gamma Conversion mode by default for all renderers, but still allow to override it via config dialog
        Ogre::RenderSystemList::const_iterator pRend;
        for (pRend = mRoot->getAvailableRenderers().begin(); pRend != mRoot->getAvailableRenderers().end(); ++pRend)
        {
            Ogre::RenderSystem* rs = *pRend;
            rs->setConfigOption("sRGB Gamma Conversion", "Yes");
        }

        if( mAlwaysAskForConfig || !mRoot->restoreConfig() )
        {
            if( !mRoot->showConfigDialog() )
            {
                mQuit = true;
                return;
            }
        }

        mRoot->initialise( false, windowTitle );

        Ogre::ConfigOptionMap& cfgOpts = mRoot->getRenderSystem()->getConfigOptions();

        int width   = 1280;
        int height  = 720;

        Ogre::ConfigOptionMap::iterator opt = cfgOpts.find( "Video Mode" );
        if( opt != cfgOpts.end() )
        {
            //Ignore leading space
            const Ogre::String::size_type start = opt->second.currentValue.find_first_of("012356789");
            //Get the width and height
            Ogre::String::size_type widthEnd = opt->second.currentValue.find(' ', start);
            // we know that the height starts 3 characters after the width and goes until the next space
            Ogre::String::size_type heightEnd = opt->second.currentValue.find(' ', widthEnd+3);
            // Now we can parse out the values
//             width   = Ogre::StringConverter::parseInt( opt->second.currentValue.substr( 0, widthEnd ) );
//             height  = Ogre::StringConverter::parseInt( opt->second.currentValue.substr(
//                                                            widthEnd+3, heightEnd ) );
        }
        Ogre::NameValuePairList params;
        bool fullscreen = Ogre::StringConverter::parseBool( cfgOpts["Full Screen"].currentValue );
    #if OGRE_USE_SDL2
        int screen = 0;
        int posX = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);
        int posY = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);

        if(fullscreen)
        {
            posX = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
            posY = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
        }

        mSdlWindow = SDL_CreateWindow(
                    windowTitle.c_str(),    // window title
                    posX,               // initial x position
                    posY,               // initial y position
                    width,              // width, in pixels
                    height,             // height, in pixels
                    SDL_WINDOW_SHOWN
                      | (fullscreen ? SDL_WINDOW_FULLSCREEN : 0) | SDL_WINDOW_RESIZABLE );

        //Get the native whnd
        SDL_SysWMinfo wmInfo;
        SDL_VERSION( &wmInfo.version );

        if( SDL_GetWindowWMInfo( mSdlWindow, &wmInfo ) == SDL_FALSE )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR,
                         "Couldn't get WM Info! (SDL2)",
                         "GraphicsSystem::initialize" );
        }

        Ogre::String winHandle;
        switch( wmInfo.subsystem )
        {
        #if defined(SDL_VIDEO_DRIVER_WINDOWS)
        case SDL_SYSWM_WINDOWS:
            // Windows code
            winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.win.window );
            break;
        #endif
        #if defined(SDL_VIDEO_DRIVER_WINRT)
        case SDL_SYSWM_WINRT:
            // Windows code
            winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.winrt.window );
            break;
        #endif
        #if defined(SDL_VIDEO_DRIVER_COCOA)
        case SDL_SYSWM_COCOA:
            winHandle  = Ogre::StringConverter::toString(WindowContentViewHandle(wmInfo));
            break;
        #endif
        #if defined(SDL_VIDEO_DRIVER_X11)
        case SDL_SYSWM_X11:
            winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.x11.window );
            break;
        #endif
        default:
            OGRE_EXCEPT( Ogre::Exception::ERR_NOT_IMPLEMENTED,
                         "Unexpected WM! (SDL2)",
                         "GraphicsSystem::initialize" );
            break;
        }

        #if OGRE_PLATFORM == OGRE_PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WINRT
            params.insert( std::make_pair("externalWindowHandle",  winHandle) );
        #else
            params.insert( std::make_pair("parentWindowHandle",  winHandle) );
        #endif
    #endif

        params.insert( std::make_pair("title", windowTitle) );
        params.insert( std::make_pair("gamma", cfgOpts["sRGB Gamma Conversion"].currentValue) );
        params.insert( std::make_pair("FSAA", cfgOpts["FSAA"].currentValue) );
        params.insert( std::make_pair("vsync", cfgOpts["VSync"].currentValue) );
        params.insert( std::make_pair("reverse_depth", "Yes" ) );

        initMiscParamsListener( params );

        mRenderWindow = Ogre::Root::getSingleton().createRenderWindow( windowTitle, width, height,
                                                                       fullscreen, &params );

        mOverlaySystem = OGRE_NEW Ogre::v1::OverlaySystem();

        setupResources();
        loadResources();
        chooseSceneManager();
        createCamera();
        mVrWorkspace = setupCompositor();

        mOvrCompositorListener->setInputType(inputType);
    #if OGRE_USE_SDL2
        mInputHandler = new SdlInputHandler(
            mSdlWindow, mCurrentGameState,
            mCurrentGameState, mCurrentGameState );
    #endif


        BaseSystem::initialize();

#if OGRE_PROFILING
        Ogre::Profiler::getSingleton().setEnabled( true );
    #if OGRE_PROFILING == OGRE_PROFILING_INTERNAL
        Ogre::Profiler::getSingleton().endProfile( "" );
    #endif
    #if OGRE_PROFILING == OGRE_PROFILING_INTERNAL_OFFLINE
        Ogre::Profiler::getSingleton().getOfflineProfiler().setDumpPathsOnShutdown(
                    mWriteAccessFolder + "ProfilePerFrame",
                    mWriteAccessFolder + "ProfileAccum" );
    #endif
#endif
    }

    void SVRGraphicsSystem::deinitialize(void)
    {
        delete mOvrCompositorListener;
        mOvrCompositorListener = 0;

        if( mVrTexture )
        {
            Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
            textureManager->destroyTexture( mVrTexture );
            mVrTexture = 0;
        }

        if( mVrCullCamera )
        {
            mSceneManager->destroyCamera( mVrCullCamera );
            mVrCullCamera = 0;
        }

        if( mHMD )
        {
            vr::VR_Shutdown();
            mHMD = NULL;
        }

        //GraphicsSystem
        BaseSystem::deinitialize();

//         saveTextureCache();
//         saveHlmsDiskCache();

        if( mCamera )
        {
            mSceneManager->destroyCamera( mCamera );
            mCamera = 0;
        }

        if( mSceneManager )
            mSceneManager->removeRenderQueueListener( mOverlaySystem );

        OGRE_DELETE mOverlaySystem;
        mOverlaySystem = 0;

    #if OGRE_USE_SDL2
        delete mInputHandler;
        mInputHandler = 0;
    #endif

        OGRE_DELETE mRoot;
        mRoot = 0;

    #if OGRE_USE_SDL2
        if( mSdlWindow )
        {
            // Restore desktop resolution on exit
            SDL_SetWindowFullscreen( mSdlWindow, 0 );
            SDL_DestroyWindow( mSdlWindow );
            mSdlWindow = 0;
        }

        SDL_Quit();
    #endif
        
    }

        //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::update( float timeSinceLast )
    {
        Ogre::WindowEventUtilities::messagePump();

    #if OGRE_USE_SDL2
        SDL_Event evt;
        while( SDL_PollEvent( &evt ) )
        {
            switch( evt.type )
            {
            case SDL_WINDOWEVENT:
                handleWindowEvent( evt );
                break;
            case SDL_QUIT:
                mQuit = true;
                break;
            default:
                break;
            }

            mInputHandler->_handleSdlEvents( evt );
        }
    #endif
        BaseSystem::update( timeSinceLast );

        if( mRenderWindow->isVisible() )
            mQuit |= !mRoot->renderOneFrame();

        mAccumTimeSinceLastLogicFrame += timeSinceLast;

        //SDL_SetWindowPosition( mSdlWindow, 0, 0 );
        /*SDL_Rect rect;
        SDL_GetDisplayBounds( 0, &rect );
        SDL_GetDisplayBounds( 0, &rect );*/
    }

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::stopCompositor(void)
    {
        if( mVrWorkspace )
        {
            Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
            compositorManager->removeWorkspace( mVrWorkspace );
            mVrWorkspace = 0;
        }
    }
    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::restartCompositor(void)
    {
        stopCompositor();
        mVrWorkspace = setupCompositor();
    }
    
    //-----------------------------------------------------------------------------------
    bool SVRGraphicsSystem::isWriteAccessFolder(
        const Ogre::String &folderPath,
        const Ogre::String &fileToSave )
    {
        if( !Ogre::FileSystemLayer::createDirectory( folderPath ) )
            return false;

        std::ofstream of( (folderPath + fileToSave).c_str(),
                          std::ios::out | std::ios::binary | std::ios::app );
        if( !of )
            return false;

        return true;
    }

    //-----------------------------------------------------------------------------------
#if OGRE_USE_SDL2
    void SVRGraphicsSystem::handleWindowEvent( const SDL_Event& evt )
    {
        switch( evt.window.event )
        {
            /*case SDL_WINDOWEVENT_MAXIMIZED:
                SDL_SetWindowBordered( mSdlWindow, SDL_FALSE );
                break;
            case SDL_WINDOWEVENT_MINIMIZED:
            case SDL_WINDOWEVENT_RESTORED:
                SDL_SetWindowBordered( mSdlWindow, SDL_TRUE );
                break;*/
            case SDL_WINDOWEVENT_SIZE_CHANGED:
                int w,h;
                SDL_GetWindowSize( mSdlWindow, &w, &h );
#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
                mRenderWindow->requestResolution( w, h );
#endif
                mRenderWindow->windowMovedOrResized();
                break;
            case SDL_WINDOWEVENT_RESIZED:
#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
                mRenderWindow->requestResolution( evt.window.data1, evt.window.data2 );
#endif
                mRenderWindow->windowMovedOrResized();
                break;
            case SDL_WINDOWEVENT_CLOSE:
                break;
        case SDL_WINDOWEVENT_SHOWN:
            mRenderWindow->_setVisible( true );
            break;
        case SDL_WINDOWEVENT_HIDDEN:
            mRenderWindow->_setVisible( false );
            break;
        case SDL_WINDOWEVENT_FOCUS_GAINED:
            mRenderWindow->setFocused( true );
            break;
        case SDL_WINDOWEVENT_FOCUS_LOST:
            mRenderWindow->setFocused( false );
            break;
        }
    }
#endif

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::processIncomingMessage( Mq::MessageId messageId, const void *data )
    {
    }

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::addResourceLocation(
        const Ogre::String &archName,
        const Ogre::String &typeName,
        const Ogre::String &secName )
    {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    archName, typeName, secName);
    }

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::setupResources(void)
    {
        // Load resource paths from config file
        Ogre::ConfigFile cf;
        cf.load(mResourcePath + "resources2.cfg");
        // Go through all sections & settings in the file
        Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

        Ogre::String secName, typeName, archName;
        while( seci.hasMoreElements() )
        {
            secName = seci.peekNextKey();
            Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();

            if( secName != "Hlms" )
            {
                Ogre::ConfigFile::SettingsMultiMap::iterator i;
                for (i = settings->begin(); i != settings->end(); ++i)
                {
                    typeName = i->first;
                    archName = i->second;
                    addResourceLocation( archName, typeName, secName );
                }
            }
        }
//         Ogre::ConfigFile cf;
//         cf.load(mResourcePath + "resources2.cfg");

        Ogre::String dataFolder = cf.getSetting( "DoNotUseAsResource", "Hlms", "" );

        if( dataFolder.empty() )
            dataFolder = "./";
        else if( *(dataFolder.end() - 1) != '/' )
            dataFolder += "/";

        Ogre::String originalDataFolder = cf.getSetting( "DoNotUseAsResource", "Hlms", "" );

        if( originalDataFolder.empty() )
            originalDataFolder = "./";
        else if( *(originalDataFolder.end() - 1) != '/' )
            originalDataFolder += "/";

        const char *c_locations[] =
        {
            "Hlms/Common/GLSL",
            "Hlms/Common/HLSL",
            "Hlms/Common/Metal",
            "Compute/Tools/Any",
            "Compute/VR",
            "Compute/VR/Foveated",
            "2.0/scripts/materials/PbsMaterials"
        };

        for( size_t i=0; i<sizeof(c_locations) / sizeof(c_locations[0]); ++i )
        {
            Ogre::String dataFolder1 = originalDataFolder + c_locations[i];
            addResourceLocation( dataFolder1, "FileSystem", "General" );
        }
    }

    void SVRGraphicsSystem::registerHlms(void)
    {
        Ogre::ConfigFile cf;
        cf.load( mResourcePath + "resources2.cfg" );

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE || OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
        Ogre::String rootHlmsFolder = Ogre::macBundlePath() + '/' +
                                  cf.getSetting( "DoNotUseAsResource", "Hlms", "" );
#else
        Ogre::String rootHlmsFolder = cf.getSetting( "DoNotUseAsResource", "Hlms", "" );
#endif

        if( rootHlmsFolder.empty() )
            rootHlmsFolder = "./";
        else if( *(rootHlmsFolder.end() - 1) != '/' )
            rootHlmsFolder += "/";

        //At this point rootHlmsFolder should be a valid path to the Hlms data folder

        Ogre::HlmsUnlit *hlmsUnlit = 0;

        //For retrieval of the paths to the different folders needed
        Ogre::String mainFolderPath;
        Ogre::StringVector libraryFoldersPaths;
        Ogre::StringVector::const_iterator libraryFolderPathIt;
        Ogre::StringVector::const_iterator libraryFolderPathEn;

        Ogre::ArchiveManager &archiveManager =
            Ogre::ArchiveManager::getSingleton();

        {
            //Create & Register HlmsUnlit
            //Get the path to all the subdirectories used by HlmsUnlit
            Ogre::HlmsUnlit::getDefaultPaths(
                mainFolderPath, libraryFoldersPaths );
            Ogre::Archive *archiveUnlit = archiveManager.load(
                rootHlmsFolder + mainFolderPath,
                "FileSystem", true );
            Ogre::ArchiveVec archiveUnlitLibraryFolders;
            libraryFolderPathIt = libraryFoldersPaths.begin();
            libraryFolderPathEn = libraryFoldersPaths.end();
            while( libraryFolderPathIt != libraryFolderPathEn )
            {
                Ogre::Archive *archiveLibrary =
                        archiveManager.load( rootHlmsFolder + *libraryFolderPathIt, "FileSystem", true );
                archiveUnlitLibraryFolders.push_back( archiveLibrary );
                ++libraryFolderPathIt;
            }

            //Create and register the unlit Hlms
            hlmsUnlit = OGRE_NEW Ogre::HlmsUnlit( archiveUnlit, &archiveUnlitLibraryFolders );
            Ogre::Root::getSingleton().getHlmsManager()->registerHlms( hlmsUnlit );
        }

    }

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::loadResources(void)
    {
        registerHlms();

        // Initialise, parse scripts etc
        Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups( true );
    }

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::chooseSceneManager(void)
    {
#if OGRE_DEBUG_MODE
        //Debugging multithreaded code is a PITA, disable it.
        const size_t numThreads = 1;
#else
        //getNumLogicalCores() may return 0 if couldn't detect
        const size_t numThreads = std::max<size_t>( 1, Ogre::PlatformInformation::getNumLogicalCores() );
#endif
        // Create the SceneManager, in this case a generic one
        mSceneManager = mRoot->createSceneManager( Ogre::ST_GENERIC,
                                                   numThreads,
                                                   "ExampleSMInstance" );

        mSceneManager->addRenderQueueListener( mOverlaySystem );
        mSceneManager->getRenderQueue()->setSortRenderQueue(
                    Ogre::v1::OverlayManager::getSingleton().mDefaultRenderQueueId,
                    Ogre::RenderQueue::StableSort );

        //Set sane defaults for proper shadow mapping
        mSceneManager->setShadowDirectionalLightExtrusionDistance( 500.0f );
        mSceneManager->setShadowFarDistance( 500.0f );
    }
    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::createCamera(void)
    {
        mCamera = mSceneManager->createCamera( "Main Camera" );

        // Position it at 500 in Z direction
        mCamera->setPosition( Ogre::Vector3( 0, 5, 15 ) );
        // Look back along -Z
        mCamera->lookAt( Ogre::Vector3( 0, 0, 0 ) );
        mCamera->setNearClipDistance( 0.2f );
        mCamera->setFarClipDistance( 1000.0f );
        mCamera->setAutoAspectRatio( true );
    }

    //-----------------------------------------------------------------------------------
    void SVRGraphicsSystem::initMiscParamsListener( Ogre::NameValuePairList &params )
    {
    }

    Ogre::CompositorWorkspace* SVRGraphicsSystem::setupCompositor()
    {
        mVrCullCamera = mSceneManager->createCamera( "VrCullCamera" );
        Ogre::CompositorWorkspace* vrCompositor = nullptr;
        vr::IVRCompositor *vrCompositor3D = nullptr;
        std::cout << "setupCompositor" << std::endl;
        Ogre::CompositorManager2 *compositorManager =
            mRoot->getCompositorManager2();
        initOpenVR();

        Ogre::CompositorChannelVec channels( 2u );
        channels[0] = mRenderWindow->getTexture();
        channels[1] = mVrTexture;
        vrCompositor = compositorManager->addWorkspace(
            mSceneManager, channels, mCamera,
            "SVRMirrorWindowWorkspace", true );
        vrCompositor3D =
            vr::VRCompositor();
        mOvrCompositorListener =
            new Demo::OpenVRCompositorListener(
                mHMD, vrCompositor3D, mVrTexture,
                mRoot, mVrWorkspace,
                mCamera, mVrCullCamera );
        return vrCompositor;
    }

    //-----------------------------------------------------------------------------
    // Purpose: Helper to get a string from a tracked device property and turn it
    //			into a std::string
    //-----------------------------------------------------------------------------
    std::string SVRGraphicsSystem::GetTrackedDeviceString(
        vr::TrackedDeviceIndex_t unDevice,
        vr::TrackedDeviceProperty prop,
        vr::TrackedPropertyError *peError)
    {
        vr::IVRSystem *vrSystem = vr::VRSystem();
        uint32_t unRequiredBufferLen = vrSystem->GetStringTrackedDeviceProperty( unDevice, prop,
                                                                                 NULL, 0, peError );
        if( unRequiredBufferLen == 0 )
            return "";

        char *pchBuffer = new char[ unRequiredBufferLen ];
        unRequiredBufferLen = vrSystem->GetStringTrackedDeviceProperty(
            unDevice, prop, pchBuffer,
            unRequiredBufferLen, peError );
        std::string sResult = pchBuffer;
        delete [] pchBuffer;
        return sResult;
    }

    void SVRGraphicsSystem::initOpenVR(void)
    {
        // Loading the SteamVR Runtime
        std::cout << "initOpenVR" << std::endl;
        vr::EVRInitError eError = vr::VRInitError_None;
        mHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

        if( eError != vr::VRInitError_None )
        {
            std::cout << "initOpenVR found error" << std::endl;
            mHMD = nullptr;
//             Ogre::String errorMsg = "Unable to init VR runtime: ";
//             errorMsg += vr::VR_GetVRInitErrorAsEnglishDescription( eError );
//             OGRE_EXCEPT(
//                 Ogre::Exception::ERR_RENDERINGAPI_ERROR, errorMsg,
//                 "SVRGraphicsSystem::initOpenVR" );
        }

        mStrDriver = "No Driver";
        mStrDisplay = "No Display";

        mStrDriver = GetTrackedDeviceString(
            vr::k_unTrackedDeviceIndex_Hmd,
            vr::Prop_TrackingSystemName_String );
        mStrDisplay = GetTrackedDeviceString(
            vr::k_unTrackedDeviceIndex_Hmd,
            vr::Prop_SerialNumber_String );
        mDeviceModelNumber = GetTrackedDeviceString(
            vr::k_unTrackedDeviceIndex_Hmd,
            vr::Prop_ModelNumber_String );

        initCompositorVR();

        uint32_t width, height, new_width;
        //gives us the render target off one eye
        mHMD->GetRecommendedRenderTargetSize( &width, &height );

        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
        //Radial Density Mask requires the VR texture to be UAV & reinterpretable
        mVrTexture = textureManager->createOrRetrieveTexture(
            "OpenVR Both Eyes",
            Ogre::GpuPageOutStrategy::Discard,
            Ogre::TextureFlags::RenderToTexture|
            Ogre::TextureFlags::Uav|
            Ogre::TextureFlags::Reinterpretable,
            Ogre::TextureTypes::Type2D );
        new_width = width << 1u;
        mVrTexture->setResolution( new_width, height);
        mVrTexture->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
        mVrTexture->scheduleTransitionTo(
            Ogre::GpuResidency::Resident );

        std::cout << "compositor" << std::endl;
        Ogre::CompositorManager2 *compositorManager =
            mRoot->getCompositorManager2();
        mVrWorkspace = compositorManager->addWorkspace(
            mSceneManager, mVrTexture,
            mCamera, "SVRWorkspace", true, 0 );
        std::cout << "compositor after" << std::endl;

        createHiddenAreaMeshVR();
    }

    void SVRGraphicsSystem::initCompositorVR(void)
    {
        if ( !vr::VRCompositor() )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_RENDERINGAPI_ERROR,
                         "VR Compositor initialization failed. See log file for details",
                         "SVRGraphicsSystem::initCompositorVR" );
        }
    }

    void SVRGraphicsSystem::createHiddenAreaMeshVR(void)
    {
        try
        {
            Ogre::ConfigFile cfgFile;
            cfgFile.load( mResourcePath + "HiddenAreaMeshVr.cfg" );

            Ogre::HiddenAreaVrSettings setting;
            setting = Ogre::HiddenAreaMeshVrGenerator::loadSettings( mDeviceModelNumber, cfgFile );
            if( setting.tessellation > 0u )
                Ogre::HiddenAreaMeshVrGenerator::generate( "HiddenAreaMeshVr.mesh", setting );
            Ogre::LogManager &logManager = Ogre::LogManager::getSingleton();
            logManager.logMessage( "HiddenAreaMeshVR optimization IS AVAILABLE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );
        }
        catch( Ogre::FileNotFoundException &e )
        {
            Ogre::LogManager &logManager = Ogre::LogManager::getSingleton();
            logManager.logMessage( e.getDescription() );
            logManager.logMessage( "HiddenAreaMeshVR optimization won't be available" );
        }
    }

};
