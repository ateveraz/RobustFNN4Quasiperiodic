//
// Created by ateveraz on 19/11/24.
//

#ifndef ADAPINTEGRALGAIN_H
#define ADAPINTEGRALGAIN_H

#include <iostream>
#include <Eigen/Dense>

class AdapIntegralGain {
    public:
        AdapIntegralGain();
        ~AdapIntegralGain();

        void ResetGain();
        void setAdaptiveIntegralGain(const Eigen::Matrix3f gamma0_, const float gamma1_);
        Eigen::Vector3f computeIntegralTerm(const Eigen::Vector3f& nu, const Eigen::Vector3f& sigma, const float dt);


    private:
		Eigen::Matrix3f  Gamma_hat;
		Eigen::Matrix3f gamma0;
		float gamma1;

		Eigen::Matrix3f UpdateIntegralGain(const Eigen::Vector3f& nu, const Eigen::Vector3f& sigma, const float dt);
};



#endif //ADAPINTEGRALGAIN_H
