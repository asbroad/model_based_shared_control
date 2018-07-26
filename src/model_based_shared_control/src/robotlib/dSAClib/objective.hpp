#ifndef OBJECTIVE_HPP
#define OBJECTIVE_HPP

#include <math.h>
#include <armadillo>
#include "wrap2Pi.hpp"
#include "../dynamicalSystems/koopman/basis_functions/basis_template.hpp"



class Objective {

public:
    arma::mat Q;
    arma::mat Qf;
    arma::mat R;
    arma::mat xd;
    Basis* basis;

    arma::mat Qfk;

    Objective(arma::mat _Q, arma::mat _R, arma::mat _Qf, arma::vec _xd, Basis* _basis) {
        Q = _Q;
        R = _R;
        Qf = _Qf;
        xd = _xd;
        basis = _basis;
        arma::vec Qfkdiag = arma::ones<arma::vec>(_basis->_nX);
        Qfk = arma::diagmat(Qfkdiag);

    }

    inline double l(const arma::vec& x, const arma::vec& u, const arma::vec& phi) {
        arma::vec xn = x;
        arma::vec fk = basis->fk(xn, u);
        return  arma::as_scalar(0.5 * (xn.t() - xd.t()) * Q * (xn - xd));
    }

    arma::vec ldx(const arma::vec& x, const arma::vec& u, const arma::vec& phi) {
        arma::vec xn = x;
        arma::vec fk = basis->fk(xn, u);
        return Q * (xn - xd);

    }

    double m(const arma::vec& x) {
        return arma::as_scalar((x.t() - xd.t()) * Qf * (x - xd) );
    }

    arma::vec mdx(const arma::vec& x) {
        return  Qf * (x - xd);
    }

    double get_cost(const arma::mat& x, const arma::mat& u, const arma::mat & phi) {
        double J = 0.0;
        for (int k = 0; k  < u.n_cols; k++ ) {
            J += l(x.col(k), u.col(k), phi.col(k));
        }
        return J + m(x.tail_cols(1));
    }

};

#endif
