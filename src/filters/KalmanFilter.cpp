#include <filters/KalmanFilter.h>

using namespace kalman::filters;

KalmanFilter::KalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension) : stateDimension(stateDimension),
controlDimension(controlDimension), observationDimension(controlDimension) {

    // initialize the state estimate with zeros
    this->currStateEstimate = Eigen::VectorXd::Zero(stateDimension);
}

void KalmanFilter::setStateEstimate(Eigen::VectorXd newState) {

    // TODO: check state dimension
    this->currStateEstimate = newState;
}

Eigen::VectorXd KalmanFilter::getPrediction() {
    // TODO
    return Eigen::VectorXd::Zero(this->stateDimension);
}

void KalmanFilter::incorporateMeasurement(Eigen::VectorXd observation) {

    // check the vector dimension
    if(observation.rows() != this->observationDimension)
        throw std::runtime_error("Mismatched observation dimension!");

    this->lastMeasurementTimestamp = std::chrono::system_clock::now();
    // TODO
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


