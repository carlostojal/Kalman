#include <filters/ExtendedKalmanFilter.h>

using namespace kalman::filters;

ExtendedKalmanFilter::ExtendedKalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension) : KalmanFilter(stateDimension, controlDimension, observationDimension) {

}
