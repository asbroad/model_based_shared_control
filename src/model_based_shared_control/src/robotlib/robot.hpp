#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <math.h>
#include <armadillo>
#include "dynamicalSystems/system.hpp"
#include "dynamicalSystems/cartpendulum.hpp"
#include "dSAClib/SAC.hpp"
#include "dSAClib/objective.hpp"

#include "dynamicalSystems/koopman/koopman_operator.hpp"
#include "dynamicalSystems/koopman/basis_functions/basis.hpp"

typedef PolynomialBasis BasisFun;

class Robot {

public:
    System* sys;
    KoopmanOperator* ksys;
    arma::vec x0;
    deiSAC* controller;
    float time_step;

    Robot(float _dt) {
        time_step = _dt;
        const float T = 1.5;
        const int N = T/time_step;
        sys = new CartPendulum(time_step);
        ksys = new KoopmanOperator(new BasisFun());
        arma::vec Qdiag = arma::zeros<arma::vec>(ksys->_nX);
        Qdiag.head_rows(2) = arma::vec({100, 0.001});
        const arma::vec xd = arma::zeros<arma::vec>(ksys->_nX);
        const arma::mat Q = arma::diagmat(Qdiag);
        arma::vec Rdiag = {0.1};
        const arma::mat R = arma::diagmat(Rdiag);
        arma::vec umax = (5)*arma::ones<arma::vec>(ksys->_nU);
        arma::vec unom = (0.01)*arma::ones<arma::vec>(ksys->_nU);
        controller = new deiSAC(ksys,
                                new Objective(Q, R, xd, new BasisFun()),
                                N, umax, unom);

    }

    ~Robot() {
        std::cout << "Deconstructing the robot" << std::endl;
        delete controller;
        delete sys;
    }

};

#endif
