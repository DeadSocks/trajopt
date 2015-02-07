/*
 * atlas_utils.cpp
 *
 */

#include "atlas_utils.hpp"



std::string convertDoubleVectortoString(std::vector<double>& v){
	std::string s = "[";
	for(std::vector<double>::iterator it = v.begin(); it != v.end(); ++it) {
		if(it != v.begin())s.append(",");
		s.append(std::to_string(*it));
	}
	return s.append("]");
}

Eigen::VectorXd linspace(double a, double b, int n) {
	Eigen::VectorXd array(n);
    double step = (b-a) / (n-1);

    for(int i =0; i < n; i ++){
    	array[i] = a;
    	a += step;
    }
    return array;
}

int traj_is_safe(trajopt::TrajArray& traj, OpenRAVE::RobotBasePtr robot, int n){
/*    Returns the set of collisions.
    manip = Manipulator or list of indices
	*/
	int collision_times = 0;

	//Interpolate trajectory:
	trajopt::TrajArray trap_up = trajopt::interp2d(linspace(0,1,n),linspace(0,1,traj.rows()),traj);

	OpenRAVE::EnvironmentBasePtr env = robot->GetEnv();

	//Testing:
/*	std::cout<< "traj " << traj <<std::endl;
	std::cout<< "traj up! " << trap_up <<std::endl;
	std::cout<< "traj size " << trap_up.rows() << " by " << trap_up.cols() <<std::endl;*/

	//Check for collisions on interpolated trajectory
	std::vector<double> res(trap_up.cols());
	for(int n = 0 ; n < trap_up.rows() ; n++)
	{
		for(int i = 0; i < trap_up.cols(); i++)
		{
			res[i] = trap_up(n,i);
		}
		robot->SetActiveDOFValues(res);
		if (env->CheckCollision(robot)) collision_times +=1;
	}
	return collision_times;
}


