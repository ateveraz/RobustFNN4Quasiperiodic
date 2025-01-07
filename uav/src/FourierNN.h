//
// Created by ateveraz on 19/11/24.
//

#ifndef FOURIERNN_H
#define FOURIERNN_H

#include <iostream>
#include <Eigen/Dense>

class FourierNN {
public:
    FourierNN();
    ~FourierNN();

    void setFourierNN(const Eigen::Matrix3f W0_, const float W1_, const float omega_, const float threshold_);
    Eigen::Vector3f computeFourierNN(Eigen::Vector3f& errorSM, float t, float delta_t);
    void ResetWeights();
    Eigen::Vector3f getMonitorValue() {return PhiMonitor.diagonal();}

    Eigen::Matrix<float, 3, 7> getWeights();

private:
    Eigen::Matrix<float, 3, 7> Weights;             // Declaración de la matriz Weights
    Eigen::Matrix<float, 7, 1> InputBase;           // Regressor
    Eigen::Vector3f Approximation;                  // Product of C*Regressor
    Eigen::Matrix3f monitor(const Eigen::Vector3f& errorSM);
    void getInputs(float t);
    Eigen::Matrix<float, 3, 7> updateWeights(const Eigen::Matrix3f& Psi, const Eigen::Vector3f& nu);

    float W1, threshold, omega;

    Eigen::Matrix3f PhiMonitor;

    Eigen::Matrix3f W0;
};

#endif //FOURIERNN_H
