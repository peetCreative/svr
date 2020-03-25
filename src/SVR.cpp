#include "OgrePrerequisites.h"
#include <iostream>

#include "OgreCommon/SdlInputHandler.h"

#include "OgreWindow.h"
#include "OgreTimer.h"

#include "Threading/OgreThreads.h"

#include "SVRGraphicsSystem.h"
#include "SVRGameState.h"


#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <pwd.h>
    #include <errno.h>
#endif

using namespace Demo;

int main( int argc, const char *argv[] )
{
    SVRGameState *graphicsGameState = 0;
    SVRGraphicsSystem *graphicsSystem = 0;

    bool config_dialog = argc >= 2 && std::strcmp(argv[1], "--config-dialog") == 0;

    createSystems( &graphicsGameState, &graphicsSystem, config_dialog );


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

        while( !graphicsSystem->getQuit() )
        {
            graphicsSystem->beginFrameParallel();
            graphicsSystem->update( static_cast<float>( timeSinceLast ) );
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
        }

        graphicsSystem->destroyScene();
        graphicsSystem->deinitialize();

        destroySystems( graphicsGameState, graphicsSystem);
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

    return 0;
}

void createSystems( SVRGameState **outGraphicsGameState,
                    SVRGraphicsSystem **outGraphicsSystem,
                    bool configDialog
                  )
{
    SVRGameState *gfxGameState = new SVRGameState(
    "SVR" );

    SVRGraphicsSystem *graphicsSystem = new SVRGraphicsSystem( gfxGameState, configDialog );

    gfxGameState->_notifyGraphicsSystem( graphicsSystem );

    *outGraphicsGameState = gfxGameState;
    *outGraphicsSystem = graphicsSystem;
}

void destroySystems( SVRGameState *graphicsGameState,
                    SVRGraphicsSystem *graphicsSystem)
{
    delete graphicsSystem;
    delete graphicsGameState;
}

const char* getWindowTitle(void)
{
    return "SVR";
}
