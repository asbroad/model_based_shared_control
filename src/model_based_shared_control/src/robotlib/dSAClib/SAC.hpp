#ifndef DEISAC_HPP
#define DEISAC_HPP

#include <armadillo>
#include "../dynamicalSystems/system.hpp"
#include "../dynamicalSystems/koopman/koopman_operator.hpp"

#include "objective.hpp"
#include <math.h>

class deiSAC {

private:

    int _N;
    arma::mat uArr;
    arma::vec rho0;
    std::vector<arma::vec> xhist;
    std::vector<arma::vec> xpred;
    arma::vec umax;
    arma::vec unom;
    arma::mat phiArr;

public:
    Objective* obj;

    KoopmanOperator* sys;

    deiSAC(KoopmanOperator* _sys, Objective* _obj, int N, arma::vec _umax, arma::vec _unom){
        sys = _sys;
        obj = _obj;
        _N = N;
        umax = _umax;
        unom = _unom;
        uArr = arma::zeros<arma::mat>(sys->_nU, _N);
        phiArr = arma::zeros<arma::mat>(sys->basis->_nX, _N);
        for (int i = 0; i < _N; i++) {
            uArr.col(i) = unom;
        }
        rho0 = arma::zeros<arma::vec>(sys->nX);
    }

    ~deiSAC() {
        std::cout << "Deleting controller" << std::endl;
        delete sys;
        delete obj;
    }

    arma::mat forward_sim(const arma::vec& x, const arma::mat& u);
    void forward_sim_K_sensitivity(const arma::vec& x, const arma::mat & u);
    arma::mat back_sim(const arma::mat& x);
    inline arma::vec rhok(const arma::vec&, const arma::vec&, const arma::vec&, const arma::vec &);
    arma::mat calc_ustar(const arma::mat&, const arma::mat&, const arma::mat&, double);
    arma::vec calc_dJdlam(const arma::mat&, const arma::mat&, const arma::mat&, const arma::mat&);

    arma::mat ustar_cont_search(const arma::vec& x0, const arma::mat& ustar, double Jinit) {
        int k = 1;
        double dJmin = 0;
        bool flag = false;
        double __scale = 0.2;

        arma::mat utemp = uArr + pow(__scale, k) * ustar; // copy over the ustar
        for (int i = 0; i < utemp.n_cols; i++){
            utemp.col(i) = saturate(utemp.col(i));
        }
        arma::mat xsol = forward_sim(x0, utemp); // forward simulate it
        double Jnew = obj->get_cost(xsol, utemp, phiArr);
        while (Jnew - Jinit > dJmin) {
            k++;
            utemp = uArr + pow(__scale, k) * ustar;
            for (int i = 0; i < utemp.n_cols; i++){
                utemp.col(i) = saturate(utemp.col(i));
            }
            xsol = forward_sim(x0, utemp);
            Jnew = obj->get_cost(xsol, utemp, phiArr);
            if (k > 15 ) {
                flag = true;
                break;
            }
        }
        if (flag == true) {
            return uArr;
        }
        return utemp;
    }

    arma::vec saturate(arma::vec u){
        arma::vec u_unit = arma::normalise(u);
        arma::vec uabs = arma::abs(u);
        arma::vec usign = arma::sign(u);
        for (int i = 0; i < u.n_rows; i++){
            if (uabs(i) > umax(i)){
                u = u_unit % umax;
                break;
            }
        }
        return u;
    }

    void shift_control_sequence(){
        arma::mat utemp = uArr.cols(1,_N-1);
        uArr.cols(0,_N-2) = utemp;
        uArr.col(_N-1) = unom;
    }

    arma::vec get_control(const arma::vec& x0) {

        arma::mat xsol, rhosol;
        arma::vec u_now, u2;
        int tau;
        double Jinit;

        xsol = forward_sim(x0, uArr);
        forward_sim_K_sensitivity(x0, uArr);
        rhosol = back_sim(xsol);

        Jinit = obj->get_cost(xsol, uArr, phiArr);
        double alpha_d = -555*Jinit;
        arma::mat ustar = calc_ustar(rhosol, xsol, uArr, alpha_d);
        ustar.replace(arma::datum::nan, 0);

        uArr = ustar_cont_search(x0, ustar, Jinit);
        u_now = uArr.col(0);
        shift_control_sequence();
        return u_now;
    }


};

arma::mat deiSAC::forward_sim(const arma::vec& x, const arma::mat& u) {
    xpred.clear();
    arma::mat X = arma::zeros<arma::mat>(sys->nX, _N);
    arma::vec x0 = x;
    for (int i = 0; i < _N; i++) {
        X.col(i) = x0;
        xpred.push_back(x0);
        x0 = sys->f(x0, u.col(i));
    }
    return X;
}

void deiSAC::forward_sim_K_sensitivity(const arma::vec & x, const arma::mat& u) {

    arma::vec phi0 = arma::zeros<arma::vec>(sys->basis->_nX);

    arma::vec phik = sys->basis->fkx(x);

    for (int i = 0; i < _N; i++) {
        phiArr.col(i) = phi0;
        phi0 = sys->_Kx * phi0 + phik;
    }

}

arma::mat deiSAC::back_sim(const arma::mat& x)
{
    arma::mat rho(sys->nX, _N);
    rho.col(_N-1) = obj->mdx(x.col(_N-1));
    for (int k = _N-1; k >0; k--) {
        rho.col(k-1) = rhok(rho.col(k), x.col(k), uArr.col(k-1), phiArr.col(k));
    }
    return rho;
}

inline arma::vec deiSAC::rhok(const arma::vec& rho, const arma::vec& xk, const arma::vec& uk, const arma::vec& phik)
{
    return obj->ldx(xk, uk, phik) + sys->fdx(xk, uk).t() * rho;
}

arma::mat deiSAC::calc_ustar(const arma::mat& rho, const arma::mat& x, const arma::mat& u, double alpha_d)
{
    arma::mat ustar = arma::zeros<arma::mat>(sys->nU, _N);
    arma::mat B;
    arma::mat lam;
    for (int i = 0; i < u.n_cols; i++){
        B = sys->fdu(x.col(i), u.col(i));
        lam = B.t() * rho.col(i) * rho.col(i).t() * B;
        ustar.col(i) = arma::inv(obj->R.t()) * (-B.t()*rho.col(i));
    }

    return ustar;
}

arma::vec deiSAC::calc_dJdlam(const arma::mat& ustar, const arma::mat& rho, const arma::mat& x, const arma::mat& u)
{
    arma::vec dJdlam = arma::zeros<arma::vec>(ustar.n_cols);
    arma::vec f1, f2;
    for (int i = 0; i < u.n_cols; i++){
        f1 = sys->f(x.col(i), u.col(i));
        f2 = sys->f(x.col(i), ustar.col(i));
        dJdlam(i) = arma::as_scalar(rho.col(i).t() * (f2 - f1));
    }

    return dJdlam;
}

#endif
