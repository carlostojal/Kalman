#include <filters/KalmanFilter.h>

using namespace kalman::filters;

KalmanFilter::KalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension) : stateDimension(stateDimension),
controlDimension(controlDimension), observationDimension(controlDimension) {

    // initialize the state estimate with zeros
    this->currStateEstimate = Eigen::VectorXd::Zero(stateDimension);

    // initialize the process and measurement noise with zeros
    this->processNoise = Eigen::VectorXd::Zero(stateDimension);
    this->measurementNoise = Eigen::VectorXd::Zero(observationDimension);
}

void KalmanFilter::setStateEstimate(Eigen::VectorXd newState) {

    // check state dimension
    if(newState.rows() != this->stateDimension)
        throw std::runtime_error("Mismatched state dimension!");

    this->currStateEstimate = newState;
}

Eigen::VectorXd KalmanFilter::getPrediction(Eigen::VectorXd control) {

    // check the control vector dimension
    if(control.rows() != this->controlDimension)
        throw std::runtime_error("Mismatched control dimension!");

    // ensure no two steps of the kalman filter are executed simultaneously
    std::lock_guard kalmanLock(this->kalmanMutex);

    // update the estimate from the motion and control models
    this->currStateEstimate = this->A * this->currStateEstimate + this->B * control + this->processNoise;

    // update the measurement estimate, to use it later on correction
    this->currMeasurementEstimate = this->C * this->currStateEstimate + this->measurementNoise;

    return currStateEstimate;
}

void KalmanFilter::incorporateMeasurement(Eigen::VectorXd observation) {

    // check the vector dimension
    if(observation.rows() != this->observationDimension)
        throw std::runtime_error("Mismatched observation dimension!");

    this->lastMeasurementTimestamp = std::chrono::system_clock::now();

    // TODO
    std::lock_guard kalmanLock(this->kalmanMutex);
}

void KalmanFilter::setMotionMatrix(Eigen::MatrixXd A) {
    // check the matrix dimension
    if(A.rows() != this->stateDimension || A.cols() != this->stateDimension)
        throw std::runtime_error("Mismatched motion matrix dimension!");

    this->A = A;
}

void KalmanFilter::setControlMatrix(Eigen::MatrixXd B) {
    // check the matrix dimension
    if(B.rows() != this->stateDimension || B.cols() != this->controlDimension)
        throw std::runtime_error("Mismatched control matrix dimension!");

    this->B = B;
}

void KalmanFilter::setObservationMappingMatrix(Eigen::MatrixXd C) {
    // check the matrix dimension
    if(C.rows() != this->observationDimension || C.cols() != this->stateDimension)
        throw std::runtime_error("Mismatched observation matrix dimension!");

    this->C = C;
}

void KalmanFilter::setProcessNoise(Eigen::VectorXd processNoise) {
    // check the process noise vector dimension
    if(processNoise.rows() != this->stateDimension)
        throw std::runtime_error("Mismatched process noise dimension. Must be the same as the state dimension.");

    this->processNoise = processNoise;
}

void KalmanFilter::setMeasurementNoise(Eigen::VectorXd measurementNoise) {
    // check the measurement noise vector dimension
    if(measurementNoise.rows() != this->observationDimension)
        throw std::runtime_error("Mismatched measurement noise dimenson. Must be the same as the observation dimension.");

    this->measurementNoise = measurementNoise;
}

Eigen::MatrixXd KalmanFilter::getMotionMatrix() {
    return this->A;
}

Eigen::MatrixXd KalmanFilter::getControlMatrix() {
    return this->B;
}

Eigen::MatrixXd KalmanFilter::getObservationMappingMatrix() {
    return this->C;
}

Eigen::VectorXd KalmanFilter::getProcessNoise() {
    return this->processNoise;
}

Eigen::VectorXd KalmanFilter::getMeasurementNoise() {
    return this->measurementNoise;
}


