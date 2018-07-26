#include <ros/ros.h>
#include <std_msgs/String.h>
#include <model_based_shared_control/State.h>
#include <model_based_shared_control/Control.h>
#include "robotlib/dynamicalSystems/koopman/koopman_operator.hpp"
#include "robotlib/dynamicalSystems/koopman/basis_functions/linear_basis.hpp"
#include "robotlib/dynamicalSystems/koopman/basis_functions/nonlinear_basis.hpp"
#include "robotlib/dSAClib/SAC.hpp"
#include "robotlib/dSAClib/objective.hpp"
#include "robotlib/lqr_controller.hpp"
#include <armadillo>

class HumanRobot {

public:

  // publishers and subscribers
  ros::Subscriber state_sub;
  ros::Subscriber shutdown_sub;
  ros::Publisher sac_control_pub;
  ros::Publisher lqr_control_pub;
  bool has_initialized = false;

  // messages
  model_based_shared_control::State state;
  model_based_shared_control::Control sacControl;
  model_based_shared_control::Control lqrControl;

  // data vectors
  arma::vec current_state;
  arma::vec dataIn;
  arma::vec dataOut;
  arma::vec cdataInSac, cdataInLqr;
  arma::vec hdataIn;
  arma::vec hdataOut;

  // Koopman models
  KoopmanOperator* linear_koopman_operator;
  KoopmanOperator* nonlinear_koopman_operator;

  // Controllers
  deiSAC* sacController;
  LQRController* lqrController;

  // LQR weights
  arma::mat A, B;
  arma::mat Qlqr, Rlqr;
  arma::vec unomLqr;
  arma::vec desired_state;

  HumanRobot(ros::Rate* loop_rate) {

    ros::NodeHandle nh;

    // set up publishers and subscribers
    sac_control_pub = nh.advertise<model_based_shared_control::Control>("/sac_control", 1);
    lqr_control_pub = nh.advertise<model_based_shared_control::Control>("/lqr_control", 1);
    state_sub = nh.subscribe("/state", 1, &HumanRobot::get_state, this);

    // set up data vectors
    dataIn = arma::zeros<arma::vec>(6);
    dataOut = arma::zeros<arma::vec>(6);
    cdataInSac = arma::zeros<arma::vec>(2);
    cdataInLqr = arma::zeros<arma::vec>(2);
    hdataIn = arma::zeros<arma::vec>(2);
    hdataOut = arma::zeros<arma::vec>(2);
    current_state = arma::zeros<arma::vec>(6);

    // set up Linear Koopman
    linear_koopman_operator = new KoopmanOperator(new LinearBasisFunction());

    // set up Non Linear Koopman
    nonlinear_koopman_operator = new KoopmanOperator(new NonLinearBasisFunction());

    // set up SAC controller
    arma::vec Qdiag = arma::ones<arma::vec>(nonlinear_koopman_operator->_nX);
    Qdiag[0] = 3;
    Qdiag[1] = 10;
    Qdiag[2] = 15;
    Qdiag[3] = 2;
    Qdiag[4] = 1;
    Qdiag[5] = 1;
    arma::mat Q = arma::diagmat(Qdiag);

    arma::vec Qfdiag = arma::ones<arma::vec>(nonlinear_koopman_operator->_nX);
    Qfdiag[0] = 1;
    Qfdiag[1] = 1;
    Qfdiag[2] = 1;
    Qfdiag[3] = 1;
    Qfdiag[4] = 1;
    Qfdiag[5] = 1;
    arma::mat Qf = arma::diagmat(Qfdiag);

    arma::vec Rdiag = arma::ones<arma::vec>(nonlinear_koopman_operator->_nU);
    Rdiag[0] = 1.0;
    Rdiag[1] = 1.0;
    arma::mat R = arma::diagmat(Rdiag);

    arma::vec umax = arma::ones<arma::vec>(nonlinear_koopman_operator->_nU);
    arma::vec unomSac = arma::zeros<arma::vec>(nonlinear_koopman_operator->_nU);
    desired_state = arma::zeros<arma::vec>(nonlinear_koopman_operator->_nX);

    sacController = new deiSAC(nonlinear_koopman_operator, new Objective(Q, R, Qf, desired_state, new NonLinearBasisFunction()),
                10, umax, unomSac );

    // set up LQR controller
    arma::vec Qlqrdiag = arma::zeros<arma::vec>(linear_koopman_operator->_nX);
    Qlqrdiag[0] = 1;
    Qlqrdiag[1] = 1;
    Qlqrdiag[2] = 1;
    Qlqrdiag[3] = 1;
    Qlqrdiag[4] = 1;
    Qlqrdiag[5] = 1;
    Qlqr = arma::diagmat(Qlqrdiag);

    arma::vec Rlqrdiag = arma::zeros<arma::vec>(linear_koopman_operator->_nU);
    Rlqrdiag[0] = 1.0;
    Rlqrdiag[1] = 1.0;
    Rlqr = arma::diagmat(Rlqrdiag);

    unomLqr = arma::zeros<arma::vec>(linear_koopman_operator->_nU);

    A = linear_koopman_operator->fdx(desired_state, unomLqr);
    B = linear_koopman_operator->fdu(desired_state, unomLqr);

    lqrController = new LQRController(A, B, Qlqr, Rlqr);

  }

  void compute_sac_control() {
    if (has_initialized == true) {
      cdataInSac = sacController->get_control(nonlinear_koopman_operator->basis->fkx(current_state));
      sacControl.u_1 = cdataInSac[0];
      sacControl.u_2 = cdataInSac[1];
      sac_control_pub.publish(sacControl);
    }
  }

  void compute_lqr_control() {
    if (has_initialized == true) {
      cdataInLqr = lqrController->get_control(linear_koopman_operator->basis->fkx(current_state));
      lqrControl.u_1 = cdataInLqr[0];
      lqrControl.u_2 = cdataInLqr[1];
      lqr_control_pub.publish(lqrControl);
    }
  }

  void get_state(const model_based_shared_control::State::ConstPtr& msg) {
    if (has_initialized == false) {
      dataOut[0] = msg->x;
      dataOut[1] = msg->y;
      dataOut[2] = msg->theta;
      dataOut[3] = msg->x_dot;
      dataOut[4] = msg->y_dot;
      dataOut[5] = msg->theta_dot;
      hdataOut[0] = msg->u_1;
      hdataOut[1] = msg->u_2;
      has_initialized = true;
    } else {
      if (std::abs(dataOut[0] - msg->x) + std::abs(dataOut[1] - msg->y) < 0.3){
        dataIn = dataOut;
        hdataIn = hdataOut;
        dataOut[0] = msg->x;
        dataOut[1] = msg->y;
        dataOut[2] = msg->theta;
        dataOut[3] = msg->x_dot;
        dataOut[4] = msg->y_dot;
        dataOut[5] = msg->theta_dot;
        hdataOut[0] = msg->u_1;
        hdataOut[1] = msg->u_2;
        if (msg->update == true) {
          linear_koopman_operator->gradStep(dataIn, hdataIn, dataOut, hdataOut);
          A = linear_koopman_operator->fdx(this->desired_state, this->unomLqr);
          B = linear_koopman_operator->fdu(this->desired_state, this->unomLqr);
          lqrController->compute_LQR_gain(A, B, this->Qlqr, this->Rlqr);
          nonlinear_koopman_operator->gradStep(dataIn, hdataIn, dataOut, hdataOut);
        }
        current_state = dataOut;
      } else {
        has_initialized = false;
      }
    }
  }

};

int main(int argc, char** argv) {

  ros::init(argc, argv,"human_robot");

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  HumanRobot sys(&loop_rate);

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    sys.compute_sac_control();
    sys.compute_lqr_control();
  }

  return 0;
}
