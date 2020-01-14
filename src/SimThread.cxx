/*
 * Simulation Thread
 *
 * Author: Roice (LUO Bing)
 * Date: 2017-03-15 create this file
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <cmath>
/* thread */
#include <pthread.h>
/* RAOS */
#include "model/SimModel.h" // models
#include "SimWind.h"
#include "SimThread.h"
#include "SimRecord.h"
/*file operation*/
#include <fstream>
#include <iostream>
#include <sstream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "common/csv/csv.h"
#include "ui/draw/draw_windvector.h"

#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>

float winddata_X[X_N][Y_N][Z_N] = {0};
float winddata_Y[X_N][Y_N][Z_N] = {0};
float winddata_Z[X_N][Y_N][Z_N] = {0};

bool sim_is_running = false;
double sim_time_passed = 0;
pthread_mutex_t sim_data_lock;
pthread_mutex_t data_trans_lock;

static pthread_t sim_thread_handle;
static pthread_t data_thread_handle;

typedef struct
{
    bool *exit_sim_thread;
    bool *pause_sim_thread;
} Sim_Thread_Args_t;

Sim_Thread_Args_t sim_thread_args;

typedef struct
{
    bool *exit_data_thread;
    bool *pause_data_thread;
} Data_Thread_Args_t;
Data_Thread_Args_t data_thread_args;

static bool exit_sim_thread = true;
static bool pause_sim_thread = false;

static bool exit_data_thread = true;
static bool pause_data_thread = false;

void read_csv_test(int idx);

void read_3D_csv(std::string filename, float (*A)[Y_N][Z_N], bool hasHeader);


// static SimRecord sim_record;

//=========data thread=========
#define NUM_FORWARD 3

static void *data_loop(void *args)
{
    bool *exit = ((Data_Thread_Args_t *) args)->exit_data_thread;
    bool *pause = ((Data_Thread_Args_t *) args)->pause_data_thread;

    // loop interval
    struct timespec req, rem;
    req.tv_sec = 0;
    req.tv_nsec = 1000000; // 0.001 s -> 1ms

    //double time_double = (SimModel_get_sim_state())->time;
    //  ||
    //  ||
    //  \/
    int time_int = 0;
    //  ||
    //  ||
    //  \/
    int time_loaded = 0;


    double record_time = 0.0;
    double recorded_time = 0.0;

    pthread_mutex_unlock(&data_trans_lock);

    //read_csv_test(0);

    while (!*((bool *) exit))
    {
        time_int = (int) ceil((SimModel_get_sim_state())->time);
        //printf("[wind] %dth simulation time.\n",time_int);
        //printf("[wind] time_int:[%d] time_loaded:[%d],compare time:[%d].\n",time_int,time_loaded,time_int + NUM_FORWARD);
#ifdef READ_CSV_WIND
        if (time_loaded < (time_int + NUM_FORWARD) && !*((bool *) pause))
        {
            //printf("[wind] %dth simulation time.\n", time_int);
            pthread_mutex_lock(&data_trans_lock);
            //wind data read function
            read_csv_test(floor(time_loaded / 5.0));
            //printf("[wind] %dth wind data has been loaded.\n", int(ceil(time_loaded / 5)));
            pthread_mutex_unlock(&data_trans_lock);
            time_loaded += 3;
        }
#endif
// #ifdef SAVE_RECORD_DATA
//         record_time = (SimModel_get_sim_state())->time ;
//         if(recorded_time < (record_time + 0.03448 ) && !*((bool*)pause))
//         {
//           // sim_record.savePlume();
//           recorded_time = record_time;
//         }
// #endif

        nanosleep(&req, &rem);
#ifdef DRAW_VECTOR_IN_THREAD
        WindVector_draw();
#endif
    }

    return 0;
}

bool data_start(void)
{
    // initialize lock
    pthread_mutex_init(&data_trans_lock, NULL);
    pthread_mutex_lock(&data_trans_lock);

    data_thread_args.exit_data_thread = &exit_data_thread;
    data_thread_args.pause_data_thread = &pause_data_thread;

    /* create simulation loop */
    exit_data_thread = false;
    pause_data_thread = false;
    if (pthread_create(&data_thread_handle, NULL, &data_loop, (void *) &(data_thread_args)) != 0)
        return false;
    return true;
}

void data_stop(void)
{
    if (!exit_data_thread) // to avoid close twice
    {
        // exit simulation thread
        exit_data_thread = true;
        pthread_join(data_thread_handle, NULL);
        pthread_mutex_destroy(&data_trans_lock);
        printf("[wind] Data transport thread terminated.\n");
    }
}


//=========data transport method=========
std::string wind_files_location = "../data/WindData/RAOS_Format";
bool wind_notified = false;
int current_wind_data = 0;
static bool winddata_end = false;

void read_csv_test(int idx)
{
    //For buildings
    std::string h5_filename = boost::str(boost::format("%s/data.h5") % wind_files_location);
    std::vector<std::vector<std::vector<float> > > wind_u;
    std::vector<std::vector<std::vector<float> > > wind_v;
    std::vector<std::vector<std::vector<float> > > wind_w;

    static bool already_load_wind = false;
    if (!already_load_wind)
    {
        using namespace HighFive;
        File occ_file(h5_filename, File::ReadOnly);
        DataSet u_dataset = occ_file.getDataSet("/wind-u");
        u_dataset.read(wind_u);

        DataSet v_dataset = occ_file.getDataSet("/wind-v");
        v_dataset.read(wind_v);

        DataSet w_dataset = occ_file.getDataSet("/wind-w");
        w_dataset.read(wind_w);

        for (unsigned int x_idx = 0; x_idx < wind_u.size(); x_idx ++)
        {
            for (unsigned int y_idx = 0; y_idx < wind_u[0].size(); y_idx ++)
            {
                for (unsigned int z_idx = 0; z_idx < wind_u[0][0].size(); z_idx ++)
                {
                    winddata_X[x_idx][y_idx][z_idx] = wind_u[x_idx][y_idx][z_idx];
                    winddata_Y[x_idx][y_idx][z_idx] = wind_v[x_idx][y_idx][z_idx];
                    winddata_Z[x_idx][y_idx][z_idx] = wind_w[x_idx][y_idx][z_idx];
                }
            }
        }

        already_load_wind = true;
    }
    return ;

    //time_t time1,time2;
    //std::string csv_filename = boost::str( boost::format("%s/wind0.%i.csv") % wind_files_location % idx );

    //std::string csv_filename = boost::str( boost::format("%s/wind_at_cell_center_points_%i.csv_U") % wind_files_location % idx );
    std::string X_filemane = boost::str(boost::format("%s/wind_data.%i.csv_X") % wind_files_location % idx);
    std::string Y_filemane = boost::str(boost::format("%s/wind_data.%i.csv_Y") % wind_files_location % idx);
    std::string Z_filemane = boost::str(boost::format("%s/wind_data.%i.csv_Z") % wind_files_location % idx);

    //std::string X_filemane = boost::str( boost::format("%s/wind_data.csv_X") % wind_files_location );
    //std::string Y_filemane = boost::str( boost::format("%s/wind_data.csv_Y") % wind_files_location );
    //std::string Z_filemane = boost::str( boost::format("%s/wind_data.csv_Z") % wind_files_location );

    printf("[winddata] Wind Path: %s \n", X_filemane.c_str());

    if (FILE *file = fopen(X_filemane.c_str(), "r"))
    {
        //file exist! keep going
        fclose(file);
        printf("[winddata] Loading Wind Snapshot %i\n", idx);

        //read 3D data
        //time(&time1);
        //printf("time1 %lf\n",time1);
        read_3D_csv(X_filemane, winddata_X, false);
        read_3D_csv(Y_filemane, winddata_Y, false);
        read_3D_csv(Z_filemane, winddata_Z, false);
        //time(&time2);
        //printf("[winddata] Loading wind data %i compeleted. Use %lf seconds.\n",idx,difftime(time1,time2));
    } else if (!winddata_end)
    {
        //No more winddata, keep current winddata forever
        printf("[winddata] File %s does not exists!\n", X_filemane.c_str());
        printf("[winddata] No more wind data available. Using last Wind snapshopt as SteadyState.\n");
        winddata_end = true;
    }

    /*
    io::CSVReader<3> in(csv_filename.c_str());
    in.read_header(io::ignore_extra_column,"a","b","sum");
    if(!in.has_column("a")||!in.has_column("b"))
      printf("Not found a or b column \n");
    bool has_sum = in.has_column("sum");
    int a,b,sum;
    printf("a + b = sum\n");
    while(in.read_row(a,b,sum)){
      if(has_sum){
        sum = a + b;
        printf("%d + %d = %d\n",a,b,sum);
      }
    }
    */

}


void read_3D_csv(std::string filename, float (*A)[Y_N][Z_N], bool hasHeader = false)
{
    //open file
    //printf("1");
    std::ifstream infile(filename.c_str());
    std::string line;
    int line_counter = 0;

    //If header -> read 4 Header lines & configure all matrices to given dimensions!
    //if (hasHeader)
    //{
    //}
    //Read file line by line
    int x_idx = 0;
    int y_idx = 0;
    int z_idx = 0;

    while (std::getline(infile, line))
    {
        line_counter++;
        std::stringstream ss;
        ss << line;
        if (z_idx >= 1)
        {
            //printf("Trying to read:[%s]",line.c_str());
        }

        if (line == ";")
        {
            //New Z-layer
            z_idx++;
            x_idx = 0;
            y_idx = 0;
        } else
        {   //New line with constant x_idx and all the y_idx values
            while (!ss.fail())
            {
                double f;
                ss >> f;        //get one double value
                if (!ss.fail())
                {
                    A[x_idx][y_idx][z_idx] = f;
                    y_idx++;
                }
            }

            //Line has ended
            x_idx++;
            y_idx = 0;
        }
    }
    //printf("[winddata] End of File.\n");

}


/*
// void read_wind_data(int idx)
// {
//     std::string wind_filename = boost::str( boost::format("%s.%i.csv") % wind_files_location % idx );
//     printf("[wind] Reading Wind Data File : %s\n",wind_filename.c_str());
//
//     if(FILE *file = fopen(wind_filename.c_str(),"r"))
//     {
//
//     }
//     else
//     {
//         //No more wind data.Keep current info.
//         if(!wind_notified)
//         {
//             printf("[wind] File %s Does Not Exists!",wind_filename.c_str());
//             printf("[wind] No more wind data avallibale.Using the last wind data as steadystate");
//             wind_notified = true;
//         }
//     }
//
//     //Update the current idx
// 	current_wind_data = idx;
// }
//
// void read_csv_file(std::string filename,std::vector< std::vector< std::vector<double> > > &A, bool hasHeader=false)
// {
// }
*/

//=========simulation thread=========
static void *sim_loop(void *args)
{
    bool *exit = ((Sim_Thread_Args_t *) args)->exit_sim_thread;
    bool *pause = ((Sim_Thread_Args_t *) args)->pause_sim_thread;
    struct timespec req, rem;
    int save_count = 0;

    // loop interval
    req.tv_sec = 0;
    req.tv_nsec = 1000000; // 0.001 s

    /* initialize simulator stuff (quadrotor, plume, etc.) */
    SimModel_init();
    sim_time_passed = 0;
    sim_is_running = true;
    pthread_mutex_unlock(&sim_data_lock);

    while (!*((bool *) exit))
    {
        if (!*((bool *) pause))
        {//if not pause
            // update models
            pthread_mutex_lock(&sim_data_lock);
            SimModel_update();
            pthread_mutex_unlock(&sim_data_lock);
        }

        // if (save_count%1000 == 0)
        // {
        //   pthread_mutex_lock(&sim_data_lock);
        //   SimSaveData();
        //   pthread_mutex_unlock(&sim_data_lock);
        // }


        nanosleep(&req, &rem);
    }

    sim_is_running = false;
    /* Save data */
    SimSaveData();
    // delete events, free memory...
    SimModel_destroy();

    return 0;
}

bool sim_start(void)
{
    // initialize lock
    pthread_mutex_init(&sim_data_lock, NULL);
    pthread_mutex_lock(&sim_data_lock);

    sim_thread_args.exit_sim_thread = &exit_sim_thread;
    sim_thread_args.pause_sim_thread = &pause_sim_thread;

    /* create simulation loop */
    exit_sim_thread = false;
    pause_sim_thread = false;
    if (pthread_create(&sim_thread_handle, NULL, &sim_loop, (void *) &(sim_thread_args)) != 0)
        return false;

    return true;
}

void sim_stop(void)
{
    if (!exit_sim_thread) // to avoid close twice
    {
        // exit simulation thread
        exit_sim_thread = true;
        pthread_join(sim_thread_handle, NULL);
        pthread_mutex_destroy(&sim_data_lock);
        printf("Simulation thread terminated.\n");
    }
}



//=========information method=========
bool sim_is_running_or_not(void)
{
    return sim_is_running;
}

double sim_get_time_passed(void)
{
    return sim_time_passed;
}

pthread_mutex_t *sim_get_data_lock(void)
{
    return &sim_data_lock;
}

bool *sim_get_pauss_control_addr(void)
{
    return &pause_sim_thread;
}
