#ifndef KOOPMAN_OPERATOR_HPP
#define KOOPMAN_OPERATOR_HPP

#include <armadillo>
#include "../system.hpp"
#include "basis_functions/basis_template.hpp"
#include <math.h>


class KoopmanOperator : public System
{

private:
    arma::mat _A;
    arma::mat _G;
    arma::mat _Ktrans;
public:
    arma::mat _K;
    arma::mat _Kx;
    arma::mat _Ku;

    int _nK;
    int _nM;
    int _nX;
    int _nU;
    int _nKU;

    Basis* basis;

    KoopmanOperator(Basis* _basis) : System(_basis->_nX, _basis->_nU) {
        basis = _basis;
        _nK = basis->_nK;
        _nX = basis->_nX;
        _nU = basis->_nU;
        _nM = basis->_nM;
        _nKU = basis->_nKU;
        _A = arma::zeros<arma::mat>(_nK, _nK);
        _G = arma::zeros<arma::mat>(_nK, _nK);
        _K = arma::zeros<arma::mat>(_nK, _nK);
        _Kx = 0.1*arma::ones<arma::mat>(_nX, _nX);
        _Ku = 0.1*arma::ones<arma::mat>(_nX, _nKU);

        _Ktrans = _K.t();
    }

    inline arma::vec f(const arma::vec& x, const arma::vec& u) {
        if (x.n_rows < _nX) {
            return _Kx * basis->fkx(x) + _Ku * basis->fku(x, u);
        } else {
            arma::vec fkx = basis->fkx(x.head_rows(_nM));
            arma::vec fku = basis->fku(x.head_rows(_nM), u);
            return _Kx * fkx + _Ku * fku;
        }
    }


    inline arma::mat fdx(const arma::vec& x, const arma::vec& u) {
        return _Kx;
    }
    inline arma::mat fdu(const arma::vec& x, const arma::vec& u) {
        return _Ku * basis->fkudu(x.head_rows(_nM), u);
    }

    void gradStep(const arma::vec& dataIn, const arma::vec& hdataIn, const arma::vec& dataOut, const arma::vec& hdataOut) {

        arma::vec phix = basis->fk(dataIn, hdataIn);
        arma::vec phixpo = basis->fk(dataOut, hdataOut);

        _G += phix * phix.t();
        _A += phix * phixpo.t();

        try {
            _K = arma::pinv(_G) * _A;
            _Ktrans = _K.t();
            _Kx = _Ktrans.submat(0, 0, _nX-1, _nX-1);
            _Ku = _Ktrans.submat(0, _nX, _nX-1, _nX + _nKU - 1);
        } catch (std::runtime_error& e) {
            std::cout << "CAUGHT ERROR" << std::endl;
        }

    }

    void computeOperator() {
      try {
            _K = arma::pinv(_G) * _A;
            _Ktrans = _K.t();
            _Kx = _Ktrans.submat(0, 0, _nX-1, _nX-1);
            _Ku = _Ktrans.submat(0, _nX, _nX-1, _nX + _nKU - 1);
        } catch (std::runtime_error& e) {
            std::cout << "CAUGHT ERROR" << std::endl;
        }
    }

    void saveOperator(std::string filePath) {
      _Ktrans.save(filePath + ".csv", arma::raw_ascii);
      _Ktrans.save(filePath + ".bin");
    }

    void loadOperator(std::string filePath) {
      _Ktrans.load(filePath);
      _K = _Ktrans.t();
      _Kx = _Ktrans.submat(0, 0 ,_nX-1, _nX-1);
      _Ku = _Ktrans.submat(0, _nX, _nX-1, _nX + _nKU-1);
      std::cout << "Loaded Koopman Operator." << std::endl;
    }

    void showOperator() {
        std::cout << _K.t() << std::endl;
    }
};


#endif
