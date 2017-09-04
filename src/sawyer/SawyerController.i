/*
 * SawyerController.i
 *
 *  Created on: Sept 1, 2017
 *      Author: Holly Liang
 */

%module SawyerController
%{
    /* Put header files here or function declarations like below */
    #include "SawyerController.h"
    /*extern bool SawyerController::getRunloop();
    extern void SawyerController::stop(int);
    extern void SawyerController::readFromRedis();
    extern void SawyerController::writeToRedis();
    extern void SawyerController::calcTorque();*/
%}
%include <std_string.i>
%include <std_vector.i>

%template(IntVector) std::vector<int>;
%template(DoubleVector) std::vector<double>;

%include "SawyerController.h"

