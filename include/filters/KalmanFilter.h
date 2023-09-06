#ifndef KALMAN_KALMAN_FILTER_H_
#define KALMAN_KALMAN_FILTER_H_

#include <Eigen/Dense>

namespace kalman::filters {
    
    /*! \brief Regular Kalman Filter implementation. */
	class KalmanFilter {

        public:
            /*! \brief Kalman filter initialization. Expects the dimension of the state vector. */
            KalmanFilter(size_t stateDimension);

        private:
            /*! \brief State vector. */
            Eigen::VectorXd stateVector;

        /*! \brief Covariance matrix. */

	};
};

#endif
