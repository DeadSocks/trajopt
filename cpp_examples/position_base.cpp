/*
 * position_base.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: odell
 */

#include <json/json.h>
#include <iostream>
#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>
#include <trajopt/problem_description.hpp>
#include <utils/interpolation.hpp>
#include "../src/osgviewer/osgviewer.hpp"
#include "atlas_utils.hpp"

//PARAMETERS
string ENV_FILE = "data/pr2test1.env.xml";
int XYZ_TARGET [3]= {0.5,0,0.9};
int QUAT_TARGET [4]= {1,0,0,0};
string LINK_NAME = "r_gripper_tool_frame";
//

}

int main(){
	OpenRAVE::RaveInitialize(true);
	OpenRAVE::EnvironmentBasePtr penv;

	penv = OpenRAVE::RaveCreateEnvironment();
	assert(penv);

	penv->SetDebugLevel(OpenRAVE::Level_Debug);

	penv->Load(ENV_FILE);
	OpenRAVE::RobotBasePtr robot = penv->GetRobot("pr2");

	assert(robot);

	std::vector<double> joints = {-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074};
	std::vector<double> xyz_target = {0.5, 0, 0.9};
	std::vector<double> quat_target = {1, 0, 0, 0}; 


	OpenRAVE::RobotBase::ManipulatorPtr rightArm = robot->GetManipulator("rightarm");

	robot->SetDOFValues(joints,1,rightArm->GetArmIndices());

	Transform init_transform = Transform(Vector(1,0,0,0),Vector(0, 0.0, 0.0));
	robot->SetTransform(init_transform);
	//robot->Set(joints);

	std::cout << convertDoubleVectortoString(joints) << std::endl;

	//Create string stream
	std::stringstream request;
	//"Fill the JSON file:
	request << "{\"basic_info\": { \
	    \"n_steps\" : 1, \
	    \"manip\" : \"active\", \
	    \"start_fixed\" : false \
	  }, \
	  \"costs\" : [\
	  {\
	    \"type\" : \"collision\",\
	    \"params\": {\"coeffs\" : [10], \"dist_pen\" : [0.025]}\
	  },\
	  ],\
	  \"constraints\" : [\
	  {\
	    \"type\" : \"pose\",\
	    \"name\" : \"final_pose\"
	    \"params\" : {
	    			\"pos_coeffs\" : [1,1,1],\
	    			\"rot_coeffs\" : [1,1,1],\
	    			\"xyz\" : "<< convertDoubleVectortoString(xyz_targ) <<",\
	                \"wxyz\" : "<< convertDoubleVectortoString(quat_targ) <<",\
					\"link\": "<< link_name <<",\
	            }\
	  }\
	  ],\
	  \"init_info\" : [\
	  {\
	  	\"type\" : \"given_traj\",\
	  	\"data\" : [1,1,1],\
	  }\
	  ]\
	}";

	 //Init val generation:

	 //Currently just using [1,1,1] in data in init_info in the json, should make rand stuff using position_base.py for example


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

	boolean success = false;

	//boost::shared_ptr<OSGViewer> _viewer;

	//_viewer.reset();

	//_viewer = OSGViewer::GetOrCreate(penv);

	//penv->Add(_viewer);

	//_viewer->main(true);

	OpenRAVE::RaveDestroy();
	return 0;
}


