/*
 * arm_to_joint_target.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: odell
 */

#include <json/json.h>
#include <iostream>
#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>
//#include <trajopt/problem_description.hpp>
//#include <../src/utils/interpolation.hpp>
#include "../src/osgviewer/osgviewer.hpp"
#include "atlas_utils.hpp"


int main(){
	OpenRAVE::RaveInitialize(true);
	OpenRAVE::EnvironmentBasePtr penv;

	penv = OpenRAVE::RaveCreateEnvironment();
	assert(penv);

	penv->SetDebugLevel(OpenRAVE::Level_Debug);

	penv->Load("robots/pr2-beta-static.zae");
	penv->Load("../data/table.xml");
	OpenRAVE::RobotBasePtr robot = penv->GetRobot("pr2");

	assert(robot);

	std::vector<double> joint_start = {-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074};

	OpenRAVE::RobotBase::ManipulatorPtr rightArm = robot->GetManipulator("rightarm");

	robot->SetDOFValues(joint_start,1,rightArm->GetArmIndices());
	//robot->Set(joints);

	std::vector<double> joint_target = {0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988};

	std::cout << convertDoubleVectortoString(joint_start) << std::endl;
	//I'm not actually sure if we want to print this or not.

	//Create string stream
	std::stringstream request;
	//"Fill the JSON file:

	/*
	Add this line: 
	      \"continuous\" : true,\
	to the collision parameters if you want continuity testing.
	*/
	
	request << "{\"basic_info\": { \
	    \"n_steps\" : 10, \
	    \"manip\" : \"rightarm\", \
	    \"start_fixed\" : true \
	  }, \
	  \"costs\" : [\
	  {\
	    \"type\" : \"joint_vel\",\
	    \"params\": {\"coeffs\" : [1]}\
	  },\
	  {\
	    \"type\" : \"collision\",\
	    \"params\" : {\
	      \"coeffs\" : [20],\
	      \"dist_pen\" : [0.025]\
	    }\
	  }\
	  ],\
	  \"constraints\" : [\
	  {\
	    \"type\" : \"joint\",\
	    \"params\" : {\"vals\" : "<< convertDoubleVectortoString(joint_target) <<" }\
	  }\
	  ],\
	  \"init_info\" : {\
	      \"type\" : \"straight_line\",\
	      \"endpoint\" : "<< convertDoubleVectortoString(joint_target) <<" \
	  }\
	}";


	 // Parsing:
	 Json::Value root;
	 Json::Reader reader;
	 bool parsedSuccess = reader.parse(request,
	                                   root,
	                                   false);

	 if(not parsedSuccess)
	 {
		 std::cerr<<"Failed to parse the JSON"<<std::endl
	       <<reader.getFormatedErrorMessages()
	       <<std::endl;
	   return 1;
	 }

	std::cout<<request<<std::endl;

	trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(root, penv);

	trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(prob, true);



	//Collision checker function:
	std::cout<< "Number of collisions: " << traj_is_safe(result->traj,robot) <<std::endl;

	std::vector<double> res(result->traj.cols());
	for(int i = 0; i < result->traj.cols(); i++)
	{
		res[i] = result->traj(0,i);
		//std::cout<< "N  " << n << " res[i] " << res[i]  <<std::endl;
	}

	robot->SetActiveDOFValues(res);

	//boost::shared_ptr<OSGViewer> _viewer;
	//_viewer.reset();
	//_viewer = OSGViewer::GetOrCreate(penv);
	//penv->Add(_viewer);
	//_viewer->main(true);

	OpenRAVE::RaveDestroy();
	return 0;
}