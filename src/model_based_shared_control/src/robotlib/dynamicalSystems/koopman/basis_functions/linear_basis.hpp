#ifndef LINEAR_BASIS_HPP
#define LINEAR_BASIS_HPP

#include <armadillo>
#include <math.h>


class LinearBasisFunction : public Basis {

public:

    int _nX;
    int _nK;
    int _nU;
    int _nM;
    int _nKU;

    LinearBasisFunction(int _nX = 7, int _nK = 9, int _nU = 2 , int _nM = 6, int _nKU = 2) : Basis(_nX,_nK,_nU,_nM,_nKU) {
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
            u[1]
          });

    }

    arma::mat fkudu( const arma::vec & x, const arma::vec & u) {
        return arma::mat({
            {1,0},
            {0,1}
        });
    }

};

#endif
