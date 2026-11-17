#ifndef DRONESIM_RECORDER_H
#define DRONESIM_RECORDER_H
#include "sim_types.h"

typedef struct {
    HistorySample samples[SIM_HISTORY_CAPACITY];
    uint32_t start, count;
    uint64_t last_recorded_step;
    double sample_interval_s;
} Recorder;

void recorder_init(Recorder *recorder, double sample_interval_s);
void recorder_clear(Recorder *recorder);
void recorder_maybe_record(Recorder *recorder, const SimSnapshot *snapshot, double fixed_dt);
uint32_t recorder_count(const Recorder *recorder);
int recorder_get(const Recorder *recorder, uint32_t chronological_index, HistorySample *out);

#endif

