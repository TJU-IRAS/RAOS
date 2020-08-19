/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : virtual_plume.cpp
   Author : tao.jing
   Date   : 2020/6/19
   Brief  : 
**************************************************************************/
#include "virtual_plume.h"
#include <iostream>
#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include "robot.h"

virtual_plume::virtual_plume()
{

}

virtual_plume* virtual_plume::instance()
{
    static virtual_plume inst;
    return &inst;
}

void virtual_plume::init()
{
    float cube_length = 2.0f;
    float puff_length = 0.1f;
    auto max_virtual_puff_num = static_cast<unsigned int>(pow(( cube_length / puff_length ), 3) + 1E4);
    plumes.reserve(max_virtual_puff_num + 10); // make room
}

void virtual_plume::update_rotor_info(std::vector<Robot *> * robots)
{
    assert(robots->size() == 1);
    assert(robots->at(0)->wakes.size() == 4);

    rotor_states.clear();
    for (unsigned int idx = 0; idx < robots->at(0)->wakes.size(); idx++ )
    {
        rotor_state_t rs;
        rs.pos[0] = robots->at(0)->wakes[idx]->rotor_state.pos[0];
        rs.pos[1] = robots->at(0)->wakes[idx]->rotor_state.pos[1];
        rs.pos[2] = robots->at(0)->wakes[idx]->rotor_state.pos[2];

        rs.psi = robots->at(0)->wakes[idx]->rotor_state.psi;
        rs.thrust = robots->at(0)->wakes[idx]->rotor_state.thrust;
        rs.marker_num = 0;
        for (unsigned int wake_idx = 0;
                wake_idx < static_cast<unsigned int>(robots->at(0)->wakes[idx]->rotor_state.frame.n_blades);
                        wake_idx++ )
        {
            rs.marker_num += robots->at(0)->wakes[idx]->wake_state[wake_idx]->size();
        }
        rotor_states.emplace_back(rs);
    }
}

void virtual_plume::save_virtual_plume_info()
{
    std::cout << "Begin save cube induced velocity field..." << std::endl;
    using namespace HighFive;

    static unsigned int save_idx = 0;
    std::stringstream filename_ss;
    filename_ss << "CubeIndVel" << save_idx ++ << ".h5";
    File file(filename_ss.str(), File::ReadWrite | File::Create | File::Truncate);

    std::stringstream dataset_ss;
    dataset_ss << "CubeFrame";
    std::string dataset_name = dataset_ss.str();

    unsigned int fila_vec_len = plumes.size();
    std::cout << "Dataset name " << dataset_ss.str() << " len " << fila_vec_len << std::endl;

    std::vector<size_t> dims(2);
    dims[0] = fila_vec_len;
    dims[1] = 6;
    DataSet dataset =
        file.createDataSet<float>(dataset_name, DataSpace(dims));

    std::vector<std::vector<float>> plume_ind_vel;
    for ( auto iter : plumes)
    {
        std::vector<float> data_vec;
        for ( unsigned int idx=0; idx < sizeof(iter.pos) / sizeof(iter.pos[0]); idx ++ )
        {
            data_vec.emplace_back(iter.pos[idx]);
        }
        for ( unsigned int idx=0; idx < sizeof(iter.vel) / sizeof(iter.vel[0]); idx ++ )
        {
            data_vec.emplace_back(iter.vel[idx]);
        }
        plume_ind_vel.emplace_back(data_vec);
    }
    dataset.write(plume_ind_vel);

    std::vector<size_t> dims_info(1);
    dims_info[0] = sizeof(rotor_state_t) * rotor_states.size() / sizeof(float);
    std::cout << "Info float number " << dims_info[0] << std::endl;
    DataSet dataset_info =
        file.createDataSet<float>("Info", DataSpace(dims_info));

    // Write induced velocity related info
    std::vector<float> info_vec;
    for (auto iter : rotor_states)
    {
        for( unsigned int idx = 0; idx < 3; idx ++ )
        {
            info_vec.emplace_back(iter.pos[idx]);
        }
        info_vec.emplace_back(iter.psi);
        info_vec.emplace_back(iter.thrust);
        info_vec.emplace_back(iter.marker_num);
    }
    dataset_info.write(info_vec);

    std::cout << "Finish save cube induced velocity field..." << std::endl;
}