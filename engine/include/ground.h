#ifndef DRONESIM_GROUND_H
#define DRONESIM_GROUND_H
#include "sim_types.h"

FlightStatus ground_resolve(RigidBodyState *body, const DroneConfig *config,
                            FlightStatus previous, double upward_force_n,
                            double weight_n, int *event_code);

#endif

