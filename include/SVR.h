#ifndef _SVR_H_
#define _SVR_H_

#include "SVRGraphicsSystem.h"
#include "SVRGameState.h"

#define LEFT 0
#define RIGHT 1

using namespace Demo;

void createSystems( SVRGameState **outGraphicsGameState,
                    SVRGraphicsSystem **outGraphicsSystem,
                    bool configDialog
                  );
void destroySystems( SVRGameState *graphicsGameState,
                     SVRGraphicsSystem *graphicsSystem);
const char* getWindowTitle(void);

#endif
