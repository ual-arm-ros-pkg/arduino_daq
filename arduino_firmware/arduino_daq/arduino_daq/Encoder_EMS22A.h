/*
 * Encoder_EMS22A.h
 *
 * Created: 20/07/2017 10:18:11
 *  Author: Francisco José Mañas
 */ 


#ifndef ENCODER_EMS22A_H_
#define ENCODER_EMS22A_H_

#pragma once

#include <stdint.h>  // uint8_t, etc.

void init_EMS22A();
uint16_t read_EMS22A();


#endif /* ENCODER_EMS22A_H_ */