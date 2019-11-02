#ifndef SIM_RECORD_H
#define SIM_RECORD_H

#include "hdf5.h"
#include <vector>
#include "model/robot.h"
#include "model/SimModel.h"
#include "model/plume.h"

#include <string.h>
#include <stdlib.h>

void SimSaveData(void);
// void SimSavePlume(void);


// // #define SAVE_RECORD_DATA
//
//
// class SimRecord
// {
//   public:
//     void Init(void);
//     ~SimRecord(void);
//     void savePlume(void);
//
//
//
//   private:
//     hid_t file_id, group_id, dataset_id, dataspace_id;
//     herr_t status;
//     hsize_t data_dims[2]; //2D array
//
//     float* data;
//
//     char filename[60];
//     int plume_save_idx = 1;
//
//     std::vector<Robot*>* robots = SimModel_get_robots();
//     std::vector<FilaState_t>* plume = plume_get_fila_state();
// };


#endif
