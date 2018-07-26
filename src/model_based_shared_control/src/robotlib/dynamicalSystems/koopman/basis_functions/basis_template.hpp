#ifndef BASIS_TEMPLATE_HPP
#define BASIS_TEMPLATE_HPP

#include <armadillo>

class Basis {

public:

    int _nX;
    int _nK;
    int _nM;
    int _nU;
    int _nKU;
    Basis(int _nX, int _nK, int _nU, int _nM, int _nKU) : _nX(_nX), _nK(_nK), _nU(_nU), _nM(_nM), _nKU(_nKU) { ; }
    virtual ~Basis() { ; }
    virtual arma::vec fk(const arma::vec& x, const arma::vec& u) = 0;
    virtual arma::vec fkx(const arma::vec& x) = 0;
    virtual arma::vec fku(const arma::vec& x, const arma::vec& u) = 0;
    virtual arma::mat fkudu(const arma::vec& x, const arma::vec& u) = 0;

};

#endif
