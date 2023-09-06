#ifndef KALMAN_KALMAN_FILTER_H_
#define KALMAN_KALMAN_FILTER_H_

#include <Eigen/Dense>
#include <chrono>

namespace kalman::filters {
    
    /*! \brief Regular Kalman Filter implementation. */
	class KalmanFilter {

        public:
            /*! \brief Kalman filter initialization. Expects the dimension of the state vector. */
            KalmanFilter(size_t stateDimension);

            // TODO: how to incorporate the controls generically
            /*! \brief Get the next state prediction from the system dynamics model and controls. Updates the error covariance. */
            Eigen::VectorXd getPrediction();

            // TODO: how to incorporate the measurements generically
            /*! \brief Incorporate a measurement to correct the state. Updates error covariance. */
            void incorporateMeasurement();


        private:

            const size_t stateDimension;

            /*! \brief State vector. */
            Eigen::VectorXd currStateEstimate;

            /*! \brief Last measurement timestamp. */
            std::chrono::time_point<std::chrono::system_clock> lastMeasurementTimestamp;

            // TODO: covariance matrices

	};
};

#endif
