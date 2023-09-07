#include <filters/KalmanFilter.h>

using namespace kalman::filters;

KalmanFilter::KalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension) : stateDimension(stateDimension),
controlDimension(controlDimension), observationDimension(observationDimension) {

    // initialize the state estimate with zeros
    this->currStateEstimate = Eigen::VectorXd::Zero(stateDimension);

    // initialize the predicted state covariance
    this->P = Eigen::MatrixXd::Zero(stateDimension, stateDimension);

    // initialize the motion/process covariance
    this->R = Eigen::MatrixXd::Zero(stateDimension, stateDimension);

    // initialize the observation/measurement covariance
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

    // ----- PREDICTION STEP -----

    // update the estimate from the motion and control models
    this->currStateEstimate = this->A * this->currStateEstimate + this->B * control;

    // update the state covariance 
    this->P = this->A * this->P * this->A.transpose() + this->R;

    return currStateEstimate;
}

void KalmanFilter::incorporateMeasurement(Eigen::VectorXd observation) {

    // check the vector dimension
    if(observation.rows() != this->observationDimension)
        throw std::runtime_error("Mismatched observation dimension!");

    this->lastMeasurementTimestamp = std::chrono::system_clock::now();

    std::lock_guard kalmanLock(this->kalmanMutex);

    // ----- CORRECTION STEP -----

    // compute the kalman gain
    this->K = this->P * this->C.transpose() * (this->C * this->P * this->C.transpose() + this->Q).inverse();

    // compute the corrected state estimate
    this->currStateEstimate = this->currStateEstimate + this->K * (observation - this->C * this->currStateEstimate);

    // compute the corrected state covariance
    this->P = (Eigen::MatrixXd::Identity(this->stateDimension, this->stateDimension) - this->K * this->C) * this->P;

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


