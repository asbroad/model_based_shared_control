#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <armadillo>
#include <cmath>
class LQRController {

public:

    arma::mat A;
    arma::mat B;
    arma::mat Q;
    arma::mat R;
    arma::mat P;
    arma::mat Klqr;

    int maxIter;
    float eps = 1e-3;


    LQRController(const arma::mat& _A, const arma::mat& _B,
                const arma::mat& _Q, const arma::mat& _R,
                int _maxIter = 2000, float _eps = 1e-3) {

        maxIter = _maxIter;
        eps = _eps;

        compute_LQR_gain(_A, _B, _Q, _R);
    }

    void compute_LQR_gain(const arma::mat& _A, const arma::mat& _B, const arma::mat& _Q, const arma::mat& _R) {

        A = _A;
        B = _B;
        Q = _Q;
        R = _R;
        P = _Q;

        arma::mat Pold = P;
        arma::mat delta;

        for (int i = 0; i < maxIter; i++) {
            P = A.t() * P * A - (A.t() * P * B) * arma::inv(R + B.t() * P * B) * B.t() * P * A + Q;
            delta = Pold - P;
            if (std::abs(delta.max()) < eps) {
                break;
            }
            Pold = P;
        }

        Klqr = arma::inv(R + B.t() * P * B) * B.t() * P * A;
    }

    arma::vec get_control(const arma::vec & x) {
        return -Klqr * x;
    }

};

#endif
