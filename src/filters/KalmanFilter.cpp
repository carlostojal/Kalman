#include <filters/KalmanFilter.h>

using namespace kalman::filters;

KalmanFilter::KalmanFilter(size_t stateDimension) {

    this->stateVector = Eigen::VectorXd::Zero(stateDimension);
}
