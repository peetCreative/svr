#include "SVR.h"

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

namespace Demo
{
    SVR::SVR(bool showConfigDialog)
    {
        mGameState = new SVRGameState( "SVR" );

        mGraphicsSystem =
            new SVRGraphicsSystem( mGameState, showConfigDialog );

        mGameState->_notifyGraphicsSystem( mGraphicsSystem );

        mTimer = Ogre::Timer();
        mStartTime = mTimer.getMicroseconds();

        mAccumulator = 1.0 / 60.0;
        mTimeSinceLast = 1.0 / 60.0;
    }

    SVR::~SVR()
    {
        delete mGraphicsSystem;
        delete mGameState;
    }

    bool SVR::init(InputType inputType)
    {
        try
        {
            mGraphicsSystem->initialize( inputType, getWindowTitle() );

            if( mGraphicsSystem->getQuit() )
            {
                mGraphicsSystem->deinitialize();

                std::cout << "graphics System get quit" << std::endl;
                return false; //User cancelled config
            }

            mRenderWindow = mGraphicsSystem->getRenderWindow();

            mGraphicsSystem->createScene01();
                std::cout << "after create scene" << std::endl;


        #if OGRE_USE_SDL2
            //Do this after creating the scene for easier the debugging (the mouse doesn't hide itself)
            SdlInputHandler *inputHandler = mGraphicsSystem->getInputHandler();
            inputHandler->setGrabMousePointer( true );
            inputHandler->setMouseVisible( false );
            inputHandler->setMouseRelative( true );
        #endif
        }
        catch( Ogre::Exception &e )
        {
            std::cout << "something .. oh see Ogre is brocken. AS ALWAYS.." << std::endl;
            throw e;
            return false;
        }
        catch( ... )
        {
            std::cout << "something relly bad happend" << std::endl;
            return false;
        }
        std::cout << "after init" << std::endl;
        return true;
    }

    void SVR::spin() {
        std::cout << "spin"<< std::endl;
        mGraphicsSystem->beginFrameParallel();
        mGraphicsSystem->update( static_cast<float>( mTimeSinceLast ) );
        mGraphicsSystem->finishFrameParallel();

        if( !mRenderWindow->isVisible() )
        {
            //Don't burn CPU cycles unnecessary when we're minimized.
            Ogre::Threads::Sleep( 500 );
        }

        Ogre::uint64 endTime = mTimer.getMicroseconds();
        mTimeSinceLast = (endTime - mStartTime) / 1000000.0;
        mTimeSinceLast = std::min( 1.0, mTimeSinceLast ); //Prevent from going haywire.
        mAccumulator += mTimeSinceLast;
        mStartTime = endTime;
    }

    const char* SVR::getWindowTitle(void)
    {
        return "SVR";
    }

    bool SVR::getQuit()
    {
        return mGraphicsSystem && mGraphicsSystem->getQuit();
    }
}

using namespace Demo;
int main( int argc, const char *argv[] )
{
    bool config_dialog = false;
    InputType input = VIDEO;
    for (int i = 1; i < argc; i++)
    {
        config_dialog = config_dialog || std::strcmp(argv[i], "--config-dialog") == 0;
        if (std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc)
        {
            if (std::strcmp(argv[i], "VIDEO") == 0)
                input = VIDEO;
            if (std::strcmp(argv[i], "CONST_MAT") == 0)
                input = CONST_MAT;
            if (std::strcmp(argv[i], "IMG_SERIES") == 0)
                input = IMG_SERIES;
            if (std::strcmp(argv[i], "IMG_TIMESTAMP") == 0)
                input = IMG_TIMESTAMP;
        }
    }
    SVR* svr = new SVR(config_dialog);
    if (!svr->init(input))
    {
        delete svr;
        return 1;
    }
    while (!svr->getQuit())
    {
        svr->spin();
    }
    delete svr;
    return 0;
}
