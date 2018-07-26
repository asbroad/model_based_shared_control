#include <ros/ros.h>
#include <std_msgs/String.h>
#include <model_based_shared_control/State.h>
#include <model_based_shared_control/Control.h>
#include "robotlib/dynamicalSystems/koopman/koopman_operator.hpp"
#include "robotlib/dynamicalSystems/koopman/basis_functions/linear_basis.hpp"
#include "robotlib/dynamicalSystems/koopman/basis_functions/nonlinear_basis.hpp"
#include <armadillo>

class HumanRobot {

public:

  // publishers and subscribers
  ros::Subscriber state_sub;
  ros::Subscriber shutdown_sub;
  bool has_initialized = false;

  // messages
  model_based_shared_control::State state;
  model_based_shared_control::Control control;

  // data vectors
  arma::vec current_state;
  arma::vec dataIn;
  arma::vec dataOut;
  arma::vec cdataIn;
  arma::vec hdataIn;
  arma::vec hdataOut;

  // Koopman models
  KoopmanOperator* linear_koopman_operator;
  KoopmanOperator* nonlinear_koopman_operator;

  HumanRobot(ros::Rate* loop_rate) {

    ros::NodeHandle nh;

    // set up subscribers
    state_sub = nh.subscribe("/state", 1, &HumanRobot::get_state, this);
    shutdown_sub = nh.subscribe("/shutdown", 1, &HumanRobot::get_shutdown, this);

    // set up data vectors
    dataIn = arma::zeros<arma::vec>(6);
    dataOut = arma::zeros<arma::vec>(6);
    cdataIn = arma::zeros<arma::vec>(2);
    hdataIn = arma::zeros<arma::vec>(2);
    hdataOut = arma::zeros<arma::vec>(2);
    current_state = arma::zeros<arma::vec>(6);

    // set up Linear Koopman
    linear_koopman_operator = new KoopmanOperator(new LinearBasisFunction());

    // set up Non Linear Koopman
    nonlinear_koopman_operator = new KoopmanOperator(new NonLinearBasisFunction());

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
        linear_koopman_operator->gradStep(dataIn, hdataIn, dataOut, hdataOut);
        nonlinear_koopman_operator->gradStep(dataIn, hdataIn, dataOut, hdataOut);
        current_state = dataOut;
      } else {
        has_initialized = false;
      }
		}
	}

  void get_shutdown(const std_msgs::String::ConstPtr& msg) {
    std::string filePath = msg->data;
    std::string linearFilePath = filePath + "-koopman-linear";
    std::string nonlinearFilePath = filePath + "-koopman-nonlinear";
    linear_koopman_operator->saveOperator(linearFilePath);
    nonlinear_koopman_operator->saveOperator(nonlinearFilePath);
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
  }

  return 0;
}
