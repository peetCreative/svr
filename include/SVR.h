#ifndef _SVR_H_
#define _SVR_H_

#include "OgreWindow.h"
#include "OgreTimer.h"

#define LEFT 0
#define RIGHT 1

namespace Demo
{
    class SVRGameState;
    class SVRGraphicsSystem;

    typedef enum {
        CONST_MAT,
        VIDEO,
        IMG_SERIES,
        IMG_TIMESTAMP
    } InputType;

    class SVR {
        public:
            SVR(bool showConfigDialog);
            ~SVR();
            bool init(InputType inputType);
            void spin(void);
            const char* getWindowTitle(void);
            bool getQuit();
            SVRGameState *getGameState()
            {
                return mGameState;
            };
            SVRGraphicsSystem *getGraphicsSystem()
            {
                return mGraphicsSystem;
            };
        private:
            SVRGameState *mGameState;
            SVRGraphicsSystem *mGraphicsSystem;

            Ogre::Window *mRenderWindow;
            Ogre::Timer mTimer;
            Ogre::uint64 mStartTime;

            bool mIsCameraInfoInit[2];

            double mAccumulator = 1.0 / 60.0;
            double mTimeSinceLast = 1.0 / 60.0;
            InputType mInput;
    };
}

#endif
