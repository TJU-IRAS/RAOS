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
#include "method/method.h"
#include "method/hover_measure.h"
#include "method/gas_dist_mapping.h"
#include "method/simulated_annealing_method.h"

static methodName_e current_method = METHOD_NONE;

bool method_init(methodName_e method_name)
{
    bool result = false;

    switch (method_name)
    {
        case METHOD_GAS_DIST_MAPPING:
            result = gas_dist_mapping_init();
            if (result)
                current_method = METHOD_GAS_DIST_MAPPING;
            break;
        case METHOD_HOVER_MEASURE:
            result = hover_measure_init();
            if (result)
                current_method = METHOD_HOVER_MEASURE;
            break;
        case METHOD_SIMULATED_ANNEALING:
            simulated_annealing_method::instance()->init();
            current_method = METHOD_SIMULATED_ANNEALING;
            break;
        default:
            break;
    }

    return result;
}

void method_update(SimState_t *sim_state)
{
    switch (current_method)
    {
        case METHOD_GAS_DIST_MAPPING:
            gas_dist_mapping_update(sim_state);
            break;
        case METHOD_HOVER_MEASURE:
            hover_measure_update(sim_state);
            break;
        case METHOD_SIMULATED_ANNEALING:
            simulated_annealing_method::instance()->update(sim_state);
            break;
        default:
            break;
    }
}

void method_stop(void)
{
    switch (current_method)
    {
        case METHOD_GAS_DIST_MAPPING:
            gas_dist_mapping_stop();
            current_method = METHOD_NONE;
            break;
        case METHOD_HOVER_MEASURE:
            hover_measure_stop();
            current_method = METHOD_NONE;
            break;
        case METHOD_SIMULATED_ANNEALING:
            simulated_annealing_method::instance()->stop();
            current_method = METHOD_NONE;
            break;
        default:
            break;
    }
}
