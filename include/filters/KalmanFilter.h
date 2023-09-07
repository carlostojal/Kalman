#ifndef KALMAN_KALMAN_FILTER_H_
#define KALMAN_KALMAN_FILTER_H_

#include <Eigen/Dense>
#include <chrono>
#include <mutex>

namespace kalman::filters {
    
    /*! \brief Regular Kalman Filter implementation. */
	class KalmanFilter {

        public:
            /*! \brief Kalman filter initialization. Expects the dimension of the state vector, the control vector and the observation vector. */
            KalmanFilter(size_t stateDimension, size_t controlDimension, size_t observationDimension);

            /*! \brief Hardcode a new state estimate. Use at your own risk. */
            void setStateEstimate(Eigen::VectorXd newState);

            /*! \brief Get the next state prediction from the system dynamics model and controls. Updates the error covariance. */
            Eigen::VectorXd getPrediction(Eigen::VectorXd control);

            // TODO: how to incorporate the measurements generically
            /*! \brief Incorporate a measurement/observation to correct the state. Updates error covariance. */
            void incorporateMeasurement(Eigen::VectorXd observation);

            /*! \brief Matrix of motion evolution without controls. Typically denoted as "A". */
            void setMotionMatrix(Eigen::MatrixXd A);

            /*! \brief Matrix describing how the control changes the state. Typically denoted as "B". */
            void setControlMatrix(Eigen::MatrixXd B);

            /*! \brief Matrix describing how to map the state to the observation. Typically denoted as "C". */
            void setObservationMappingMatrix(Eigen::MatrixXd C);

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

            std::mutex kalmanMutex;

            const size_t stateDimension;
            const size_t controlDimension;
            const size_t observationDimension;

            /*! \brief State vector. */
            Eigen::VectorXd currStateEstimate;

            /*! \brief Uncertainty matrix. */
            Eigen::MatrixXd currUncertainty;

            /*! \brief Matrix describing the motion evolution without controls. */
            Eigen::MatrixXd A;

            /*! \brief Matrix describing how the control changes the state. */
            Eigen::MatrixXd B;

            /*! \brief Matrix describing how to map the state to the observation. */
            Eigen::MatrixXd C;

            /*! \brief Motion/process noise matrix. */
            Eigen::MatrixXd R;

            /*! \brief Measurement/observation noise matrix. */
            Eigen::MatrixXd Q;

            /*! \brief Last measurement timestamp. */
            std::chrono::time_point<std::chrono::system_clock> lastMeasurementTimestamp;

            // TODO: covariance matrices

	};
};

#endif
