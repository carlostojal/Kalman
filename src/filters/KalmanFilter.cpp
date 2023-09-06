#include <filters/KalmanFilter.h>

using namespace kalman::filters;

KalmanFilter::KalmanFilter(size_t stateDimension) : stateDimension(stateDimension) {

    this->currStateEstimate= Eigen::VectorXd::Zero(stateDimension);
}

Eigen::VectorXd KalmanFilter::getPrediction() {
    // TODO
    return Eigen::VectorXd::Zero(this->stateDimension);
}

void KalmanFilter::incorporateMeasurement() {

    this->lastMeasurementTimestamp = std::chrono::system_clock::now();
    // TODO
}
