//
// Created by ateverz on 5/15/24.
//

#include <iostream>
#include "FourierNN.h"
#include "NMethods.h"
#include <Eigen/Dense>
//#include <type_traits>

FourierNN::FourierNN() : Weights(Eigen::Matrix<float, 3, 7>::Zero()), InputBase(Eigen::Matrix<float, 7, 1>::Zero()), Approximation(Eigen::Vector3f::Zero()) { }

FourierNN::~FourierNN() { }

void FourierNN::setFourierNN(const Eigen::Matrix3f W0_, const float W1_, const float omega_, const float threshold_)
{
    W0 = W0_;
    W1 = W1_;
    threshold = threshold_;
    omega = omega_;
}

Eigen::Matrix<float, 3, 7> FourierNN::updateWeights(const Eigen::Matrix3f& Psi, const Eigen::Vector3f& nu)
{
    return W0 * Psi * nu * InputBase.transpose() - W1 * Weights;
}

void FourierNN::ResetWeights()
{
    Weights = Eigen::Matrix<float, 3, 7>::Zero();
    InputBase = Eigen::Matrix<float, 7, 1>::Zero();
    Approximation = Eigen::Vector3f::Zero();
    PhiMonitor = Eigen::Matrix3f::Zero();
}

void FourierNN::getInputs(float t)
{
    InputBase << 1, sin(omega*t), sin(2*omega*t), sin(3*omega*t), cos(omega*t), cos(2*omega*t), cos(3*omega*t);
}

Eigen::Vector3f FourierNN::computeFourierNN(Eigen::Vector3f& errorSM, float t, float delta_t)
{
    getInputs(t);
    Eigen::Matrix3f Psi = monitor(errorSM);
    Eigen::Matrix<float, 3, 7> wp = updateWeights(Psi, errorSM);
    Weights = rk4_mat(Weights, wp, delta_t);
    Approximation = Weights * InputBase;
    PhiMonitor = monitor(errorSM);
    return PhiMonitor*Approximation;
}

Eigen::Matrix<float, 3, 7> FourierNN::getWeights()
{
    return Weights;
}

Eigen::Matrix3f FourierNN::monitor(const Eigen::Vector3f& errorSM)
{
    if (errorSM.norm() <= threshold)
    {
        return Eigen::Matrix3f::Zero();
    }
    else
    {
        return errorSM.asDiagonal();
    }
}
