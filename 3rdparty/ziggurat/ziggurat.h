#ifndef ZIGGURAT_H
#define ZIGGURAT_H

#include <stdint.h>

uint32_t cong_seeded ( uint32_t &jcong );
double cpu_time ( );
uint32_t kiss_seeded ( uint32_t &jcong, uint32_t &jsr, uint32_t &w, uint32_t &z );
uint32_t mwc_seeded ( uint32_t &w, uint32_t &z );
float r4_exp ( uint32_t &jsr, uint32_t ke[256], float fe[256], float we[256] );
void r4_exp_setup ( uint32_t ke[256], float fe[256], float we[256] );
float r4_nor ( uint32_t &jsr, uint32_t kn[128], float fn[128], float wn[128] );
void r4_nor_setup ( uint32_t kn[128], float fn[128], float wn[128] );
float r4_uni ( uint32_t &jsr );
uint32_t shr3_seeded ( uint32_t &jsr );
void timestamp ( );

#endif
