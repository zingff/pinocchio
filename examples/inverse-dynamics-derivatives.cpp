#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <cmath>

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char ** argv)
{
  using namespace pinocchio;
  
  // You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/ur_description/urdf/gen3_robotiq_2f_85.urdf") : argv[1];
  
  // Load the URDF model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  
  // Build a data related to model
  Data data(model);
  
  // Sample a random joint configuration as well as random joint velocity and acceleration
  // Eigen::VectorXd q = randomConfiguration(model);
  // for(int i = 0; i < model.nq; i++){
  //   std::cout << "q(" << i << "): " << q(i) << std::endl;
  // }
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  q(0) = cos(3.03043e-05);  // 0
  q(1) = sin(3.03043e-05);

  q(2) = 0.261897;  // 1

  q(3) = cos(3.14138);  // 2
  q(4) = sin(3.14138);
  
  q(5) = 4.01388;  // 3

  q(6) = cos(1.71222e-05);  //4 
  q(7) = sin(1.71222e-05);

  q(8) = 0.959808;  // 5
  
  q(9) = cos(1.5707);  // 6
  q(10) = sin(1.5707);



  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  
  // test
  std::cout << "nq: " << model.nq << std::endl;
  std::cout << "nv: " << model.nv << std::endl;

  // for(int i =0; i < model.nq; i++){
  //   std::cout << "nq: " << model.joints[i].nq() << std::endl;
  //   std::cout << "nv: " << model.joints[i].nv() << std::endl;

  // }


  // Allocate result container
  Eigen::MatrixXd djoint_torque_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_torque_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_torque_da = Eigen::MatrixXd::Zero(model.nv,model.nv);
  
  // Computes the inverse dynamics (RNEA) derivatives for all the joints of the robot
  rnea(model, data, q, v, a);

    // Get access to the joint torque
  std::cout << "Joint torque: " << data.tau.transpose() << std::endl;

  // crba(model, data, q);



  // // Computes the inverse dynamics (RNEA) derivatives for all the joints of the robot
  // computeRNEADerivatives(model, data, q, v, a, djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
  // // Get access to the joint torque
  // std::cout << "Joint torque: " << data.tau.transpose() << std::endl;


  

}