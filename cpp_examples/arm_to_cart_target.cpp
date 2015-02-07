/*
 * arm_to_cart_target.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: perry
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

	std::vector<double> joints = {-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074};

	OpenRAVE::RobotBase::ManipulatorPtr rightArm = robot->GetManipulator("rightarm");

	robot->SetDOFValues(joints,1,rightArm->GetArmIndices());
	//robot->Set(joints);

	std::vector<double> jointsTarghet = {-0.5468517, 1.0964348,-1.011,-1.49571816,2.2957038,-0.51354486,-1.93532726};

	std::cout << convertDoubleVectortoString(joints) << std::endl;

	//Create string stream
	std::stringstream request;
	//"Fill the JSON file:
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
	    \"name\" :\"cont_coll\",\
	    \"params\" : {\
	      \"continuous\" : true,\
	      \"coeffs\" : [20],\
	      \"dist_pen\" : [0.025]\
	    }\
	  }\
	  ],\
	  \"constraints\" : [\
	  {\
	    \"type\" : \"pose\",\
	    \"params\" : {\"xyz\" : [6.51073449e-01,-1.87673551e-01,4.91061915e-01],\
	                \"wxyz\" : [0,1,0,0],\
	                \"link\": \"r_gripper_tool_frame\",\
	                \"timestep\" : 9\
	                }\
	  }\
	  ],\
	  \"init_info\" : {\
	      \"type\" : \"straight_line\",\
	      \"endpoint\" : " << convertDoubleVectortoString(joints) << "\
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
		 std::cerr<<"Failed to parse JSON"<<std::endl
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


