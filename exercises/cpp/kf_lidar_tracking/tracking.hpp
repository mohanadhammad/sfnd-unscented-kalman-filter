#pragma once

#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.hpp"
#include "measurement_package.hpp"

class Tracking {
public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    KalmanFilter kf_;

    private:
    bool is_initialized_;
    int64_t previous_timestamp_;

    //acceleration noise components
    float noise_ax;
    float noise_ay;

};
