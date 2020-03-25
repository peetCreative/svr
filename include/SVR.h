#include "SVRGraphicsSystem.h"
#include "SVRGameState.h"

using namespace Demo;

void createSystems( SVRGameState **outGraphicsGameState,
                    SVRGraphicsSystem **outGraphicsSystem,
                    bool configDialog
                  );
void destroySystems( SVRGameState *graphicsGameState,
                     SVRGraphicsSystem *graphicsSystem);
const char* getWindowTitle(void);
