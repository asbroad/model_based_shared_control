#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <armadillo>

class System {
public:
    int nX;
    int nU;
    System(int _nX, int _nU) : nX(_nX), nU(_nU) {;}
    virtual ~System() {;}
    virtual arma::vec f(const arma::vec&, const arma::vec&) = 0;
    virtual arma::mat fdx(const arma::vec&, const arma::vec&) = 0;
    virtual arma::mat fdu(const arma::vec&, const arma::vec&) = 0;
};

#endif
