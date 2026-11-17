#ifndef DRONESIM_INTEGRATOR_H
#define DRONESIM_INTEGRATOR_H
#include "dynamics.h"

void integrator_semi_implicit_euler(RigidBodyState *body,
                                    const DynamicsOutput *derivatives, double dt);

#endif

