#include <iostream>
#include <filters/KalmanFilter.h>
#include <filters/ExtendedKalmanFilter.h>

int main() {

    kalman::filters::KalmanFilter *kf = new kalman::filters::KalmanFilter(2);
    kalman::filters::ExtendedKalmanFilter *ekf = new kalman::filters::ExtendedKalmanFilter(2);

    delete kf;
    delete ekf;

    return 0;
}
