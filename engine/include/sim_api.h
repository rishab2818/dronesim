#ifndef DRONESIM_SIM_API_H
#define DRONESIM_SIM_API_H
#include <stddef.h>
#include "drone_config.h"
#include "recorder.h"

typedef struct SimHandle SimHandle;

/** Allocates an independent simulator. Caller must call sim_destroy. */
SimHandle *sim_create(const DroneConfig *drone, const EnvironmentConfig *env);
void sim_destroy(SimHandle *sim);
SimResult sim_reset(SimHandle *sim);
SimResult sim_apply_config(SimHandle *sim, const DroneConfig *drone,
                           const EnvironmentConfig *env);
SimResult sim_set_armed(SimHandle *sim, int armed);
SimResult sim_set_mode(SimHandle *sim, FlightMode mode);
SimResult sim_set_input(SimHandle *sim, const PilotInput *input);
SimResult sim_step(SimHandle *sim, double fixed_dt_seconds);
SimResult sim_get_state(const SimHandle *sim, SimSnapshot *out);
void sim_clear_history(SimHandle *sim);
uint32_t sim_history_count(const SimHandle *sim);
SimResult sim_history_get(const SimHandle *sim, uint32_t index, HistorySample *out);
const char *sim_last_error(const SimHandle *sim);
const char *sim_result_string(SimResult result);

#endif
