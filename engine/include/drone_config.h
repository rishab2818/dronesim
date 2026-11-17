#ifndef DRONESIM_DRONE_CONFIG_H
#define DRONESIM_DRONE_CONFIG_H
#include <stddef.h>
#include "sim_types.h"

DroneConfig drone_config_default(void);
EnvironmentConfig environment_config_default(void);
SimResult drone_config_validate(const DroneConfig *drone, const EnvironmentConfig *env,
                                char *message, size_t message_capacity);
double drone_hover_omega(const DroneConfig *drone, const EnvironmentConfig *env);

#endif

