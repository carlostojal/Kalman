#ifndef KALMAN_KALMAN_FILTER_H_
#define KALMAN_KALMAN_FILTER_H_

#include <Eigen/Dense>
#include <chrono>

namespace kalman::filters {
    
    /*! \brief Regular Kalman Filter implementation. */
	class KalmanFilter {

        public:
            /*! \brief Kalman filter initialization. Expects the dimension of the state vector, the control vector and the observation vector. */
            KalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension);

            /*! \brief Hardcode a new state estimate. Use at your own risk. */
            void setStateEstimate(Eigen::VectorXd newState);

            // TODO: how to incorporate the controls generically
            /*! \brief Get the next state prediction from the system dynamics model and controls. Updates the error covariance. */
            Eigen::VectorXd getPrediction();

            // TODO: how to incorporate the measurements generically
            /*! \brief Incorporate a measurement/observation to correct the state. Updates error covariance. */
            void incorporateMeasurement(Eigen::VectorXd observation);

            /*! \brief Matrix of motion evolution without controls. Typically denoted as "A". */
            void setMotionMatrix(Eigen::MatrixXd A);

            /*! \brief Matrix describing how the control changes the state. Typically denoted as "B". */
            void setControlMatrix(Eigen::MatrixXd B);

            /*! \brief Matrix describing how to map the state to the observation. Typically denoted as "C". */
            void setObservationMappingMatrix(Eigen::MatrixXd C);

            /*! \brief Set the process noise. */
            void setProcessNoise(Eigen::VectorXd processNoise);

            /*! \brief Set the measurement noise. */
            void setMeasurementNoise(Eigen::VectorXd measurementNoise);

            /*! \brief Matrix of motion evolution without controls. Typically denoted as "A". */
            Eigen::MatrixXd getMotionMatrix();

            /*! \brief Matrix describing how the control changes the state. Typically denoted as "B". */
            Eigen::MatrixXd getControlMatrix();

            /*! \brief Matrix describing how to map the state to the observation. Typically denoted as "C". */
            Eigen::MatrixXd getObservationMappingMatrix();

            /*! \brief Process noise. */
            Eigen::VectorXd getProcessNoise();

            /*! \brief Measurement noise. */
            Eigen::VectorXd getMeasurementNoise();


        private:

            const size_t stateDimension;
            const size_t controlDimension;
            const size_t observationDimension;

            /*! \brief State vector. */
            Eigen::VectorXd currStateEstimate;

            /*! \brief Matrix describing the motion evolution without controls. */
            Eigen::MatrixXd A;

            /*! \brief Matrix describing how the control changes the state. */
            Eigen::MatrixXd B;

            /*! \brief Matrix describing how to map the state to the observation. */
            Eigen::MatrixXd C;

            /*! \brief Last measurement timestamp. */
            std::chrono::time_point<std::chrono::system_clock> lastMeasurementTimestamp;

            /*! \brief Process noise vector. */
            Eigen::VectorXd processNoise;

            /*! \brief Measurement noise vector. */
            Eigen::VectorXd measurementNoise;

            // TODO: covariance matrices

	};
};

#endif
