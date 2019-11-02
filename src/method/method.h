/*
 * Robot Active Olfation Method Gallery
 *          for RAOS simulation
 *
 * Author:
 *      Roice Luo (Bing Luo)
 * Date:
 *      2017.03.16
 */
#include "model/SimModel.h"

#define METHOD_GAS_DIST 1
//#define METHOD_HOVER 1

typedef enum
{
    METHOD_GAS_DIST_MAPPING = 0,
    METHOD_HOVER_MEASURE = 1,
    METHOD_ITEM_COUNT,
    METHOD_NONE
} methodName_e;

bool method_init(methodName_e);

void method_update(SimState_t *);

void method_stop(void);
