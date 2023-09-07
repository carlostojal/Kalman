#include <filters/KalmanFilter.h>

using namespace kalman::filters;

KalmanFilter::KalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension) : stateDimension(stateDimension),
controlDimension(controlDimension), observationDimension(controlDimension) {

    // initialize the state estimate with zeros
    this->currStateEstimate = Eigen::VectorXd::Zero(stateDimension);

    // initialize the motion/process noise matrix
    this->R = Eigen::MatrixXd::Zero(stateDimension, stateDimension);

    // initialize the observation/measurement noise matrix
    this->Q = Eigen::MatrixXd::Zero(observationDimension, observationDimension);
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
    this->currStateEstimate = this->A * this->currStateEstimate + this->B * control;

    // update the uncertainty
    this->currUncertainty = this->A * this->currUncertainty * this->A.transpose() + this->R;

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
    return this->R;
}

Eigen::VectorXd KalmanFilter::getMeasurementNoise() {
    return this->Q;
}


