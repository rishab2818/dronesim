#ifndef DRONESIM_CONTROLS_H
#define DRONESIM_CONTROLS_H
#include "sim_types.h"

PilotInput pilot_input_zero(void);
void controls_mix_manual(const PilotInput *input, double out[SIM_ROTOR_COUNT]);

#endif

