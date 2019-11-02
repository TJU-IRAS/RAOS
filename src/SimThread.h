#ifndef SIM_THREAD_H
#define SIM_THREAD_H

#include <pthread.h>

//=========data transport method=========
//void read_csv_test(void);
//function dec

#define READ_CSV_WIND 1

void read_csv_test(int idx);


//======simulation======
bool data_start(void);

void data_stop(void);

//======simulation======
bool sim_start(void);

void sim_stop(void);

bool sim_is_running_or_not(void);

pthread_mutex_t *sim_get_data_lock(void);

bool *sim_get_pauss_control_addr(void);

#endif
