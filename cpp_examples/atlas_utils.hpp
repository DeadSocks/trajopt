/*
 * utils.cpp
 *
 */

#ifndef ATLAS_UTILS_HPP
#define ATLAS_UTILS_HPP

/*** INCLUDE FILES ***/

#pragma once


#include <iostream>
#include <utils/interpolation.hpp>
#include <trajopt/problem_description.hpp>

std::string convertDoubleVectortoString(std::vector<double>& v);

Eigen::VectorXd linspace(double a, double b, int n);

int traj_is_safe(trajopt::TrajArray& traj, OpenRAVE::RobotBasePtr robot, int n = 100);


#endif // ATLAS_UTILS_HPP