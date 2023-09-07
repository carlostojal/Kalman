#ifndef KALMAN_EXTENDED_KALMAN_FILTER_H_
#define KALMAN_EXTENDED_KALMAN_FILTER_H_

#include <filters/KalmanFilter.h>

namespace kalman::filters {

    class ExtendedKalmanFilter : kalman::filters::KalmanFilter {

        public:
            /*! \brief EKF initialization. Expect the dimension of the state vector. */
            ExtendedKalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension);
    };
};

#endif
