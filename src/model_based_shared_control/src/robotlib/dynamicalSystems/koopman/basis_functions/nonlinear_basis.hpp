#ifndef NONLINEAR_BASIS_HPP
#define NONLINEAR_BASIS_HPP

#include <armadillo>
#include <math.h>


class NonLinearBasisFunction : public Basis {

public:

    int _nX;
    int _nK;
    int _nU;
    int _nM;
    int _nKU;

    NonLinearBasisFunction(int _nX = 7, int _nK = 25, int _nU = 2 , int _nM = 6, int _nKU = 18) : Basis(_nX,_nK,_nU,_nM,_nKU) {
        this->_nX = _nX;
        this->_nK = _nK;
        this->_nU = _nU;
        this->_nM = _nM;
        this->_nKU = _nKU;
    }

    arma::vec fk(const arma::vec & x, const arma::vec & u ) {
        return arma::join_cols( fkx(x), fku(x, u) );
    }

    arma::vec fkx( const arma::vec& x ) {
        return arma::vec({
          x[0],
          x[1],
          x[2],
          x[3],
          x[4],
          x[5],
          1
        });
    }

    arma::vec fku( const arma::vec & x, const arma::vec& u ) {

        return arma::vec({
            u[0],
            u[1],
            u[0]*x[0],
            u[0]*x[1],
            u[0]*x[2],
            u[0]*x[3],
            u[0]*x[4],
            u[0]*x[5],
            u[1]*x[0],
            u[1]*x[1],
            u[1]*x[2],
            u[1]*x[3],
            u[1]*x[4],
            u[1]*x[5],
            u[0]*cos(x[2]),
            u[0]*sin(x[2]),
            u[1]*cos(x[2]),
            u[1]*sin(x[2])
          });

    }

    arma::mat fkudu( const arma::vec & x, const arma::vec & u) {
        return arma::mat({
            {1,0},
            {0,1},
            {x[0],0},
            {x[1],0},
            {x[2],0},
            {x[3],0},
            {x[4],0},
            {x[5],0},
            {0,x[0]},
            {0,x[1]},
            {0,x[2]},
            {0,x[3]},
            {0,x[4]},
            {0,x[5]},
            {cos(x[2]), 0},
            {sin(x[2]), 0},
            {0, cos(x[2])},
            {0, sin(x[2])}
        });
    }

};

#endif
