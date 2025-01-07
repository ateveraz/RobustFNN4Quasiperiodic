//
// Created by ateveraz on 19/11/24.
//

#include <iostream>
#include "AdapIntegralGain.h"
#include "NMethods.h"
#include <Eigen/Dense>

AdapIntegralGain::AdapIntegralGain() : Gamma_hat(Eigen::Matrix3f::Zero()) {}

AdapIntegralGain::~AdapIntegralGain() {}

void AdapIntegralGain::ResetGain() {
    // Reset the gain
    Gamma_hat = Eigen::Matrix3f::Zero();
}

void AdapIntegralGain::setAdaptiveIntegralGain(const Eigen::Matrix3f gamma0_, const float gamma1_) {
    // Set the value of gamma0
    gamma0 = gamma0_;
    gamma1 = gamma1_;
}

Eigen::Matrix3f AdapIntegralGain::UpdateIntegralGain(const Eigen::Vector3f& nu, const Eigen::Vector3f& sigma, const float dt) {
    // Update the integral gain
    Eigen::Matrix3f Gamma_hat_p = gamma0 * nu * sigma.transpose() - gamma1*Gamma_hat;
    Gamma_hat = rk4_mat(Gamma_hat, Gamma_hat_p, dt);

    Gamma_hat.cwiseAbs();
}

Eigen::Vector3f AdapIntegralGain::computeIntegralTerm(const Eigen::Vector3f& nu, const Eigen::Vector3f& sigma, const float dt) {
    // Compute the integral term
    UpdateIntegralGain(nu, sigma, dt);
    return Gamma_hat * sigma;
}