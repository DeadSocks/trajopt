/*
 * position_base.cpp
 *  Created on: Feb 3, 2015
 *      Author: odell
 */

#include <json/json.h>
#include <iostream>
#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>
//#include <trajopt/problem_description.hpp>//Add these back if atlas_utils.hpp is removed
//#include <utils/interpolation.hpp>//see above
#include "../src/osgviewer/osgviewer.hpp"
#include "atlas_utils.hpp"

//PARAMETERS
//string ENV_FILE = "data/pr2test1.env.xml";
float XYZ_TARGET [3] = {0.5,0,0.9};
float QUAT_TARGET [4] = {1,0,0,0};
//string link_name = "r_gripper_tool_frame";
//

int main(){
	OpenRAVE::RaveInitialize(true);
	OpenRAVE::EnvironmentBasePtr penv;

	penv = OpenRAVE::RaveCreateEnvironment();
	assert(penv);

	penv->SetDebugLevel(OpenRAVE::Level_Debug);

	penv->Load("data/pr2test1.env.xml");

	OpenRAVE::RobotBasePtr robot = penv->GetRobot("pr2");

	assert(robot);

	//std::vector<double> joints = {-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074};
	std::vector<double> xyz_targ = {0.5, 0, 0.9};
	std::vector<double> quat_targ = {1, 0, 0, 0}; 
	//std::vector<double> DofVals = {-1.76080454, -0.1797336, -1.99076666, -0.21800647, 0.73899571, -0.18675692, 2.13973504, 0.14755146, 0.38328063, -0.48618576, 1.33518369};
	std::vector<double> DofVals = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

	OpenRAVE::RobotBase::ManipulatorPtr rightArm = robot->GetManipulator("rightarm");

	//robot->SetDOFValues(joints,1,rightArm->GetArmIndices());
	robot->SetDOFValues(DofVals);

	OpenRAVE::Transform init_transform = OpenRAVE::Transform(OpenRAVE::Vector(1,0,0,0),OpenRAVE::Vector(0, 0.0, 0.0));
	robot->SetTransform(init_transform);
	//robot->Set(joints);

	//std::cout << convertDoubleVectortoString(joints) << std::endl;

	//Create string stream
	std::stringstream request;

	//"Fill the JSON file:
	/*
	request << "{\"basic_info\": { \
	    \"n_steps\" : 1, \
	    \"manip\" : \"active\", \
	    \"start_fixed\" : false \
	  },\
	  \"costs\" : [\
	  {\
	    \"type\" : \"collision\",\
	    \"params\": {\"coeffs\" : [10], \
	    			\"dist_pen\" : [0.025] \
	    			}\
	  },\
	  ], \
	  \"constraints\" : [\
	  {\
	    \"type\" : \"pose\",\
	    \"name\" : \"final_pose\",\
	    \"params\" : {\"pos_coeffs\" : [1,1,1],\
	    			\"rot_coeffs\" : [1,1,1],\
	    			\"xyz\" : "<< convertDoubleVectortoString(xyz_targ) <<",\
	                \"wxyz\" : "<< convertDoubleVectortoString(quat_targ) <<",\
					\"link\": r_gripper_tool_frame\
	            }\
	  }\
	  ],\
	  \"init_info\" : {\
	  	\"type\" : \"given_traj\",\
	  	\"data\" : [-0.66 -0.04 -0.77 -1.11 -1.82 -1.49 -2.91  0.12  0.98  0.13  3.42]\
	  }\
	}";
*/
	

	
	/*
JSON v3
Error: 

  Missing '}' or object member name

0x7fffe614f218
ERROR missing field: init_info

*/

/*
request << "{\"basic_info\" : { \
	        \"n_steps\" : 1,\
	        \"manip\" : \"active\",\
	        \"start_fixed\" : false \
	    },\
	    \"costs\" : [\
	    {\
	        \"type\" : \"collision\",\
	        \"params\" : {\"coeffs\" : [10],\
	        	\"dist_pen\" : [0.025] \
	    		}\
	    }\
	    ],\
	    \"constraints\" :[\
		    {\
		        \"type\" : \"pose\",\
		        \"name\" : \"final_pose\",\
		        \"params\" : {\"pos_coeffs\" : [1,1,1],\
		            \"rot_coeffs\" : [1,1,1],\
		            \"xyz\" : "<< convertDoubleVectortoString(xyz_targ) <<",\
		            \"wxyz\" : "<< convertDoubleVectortoString(quat_targ) <<",\
		            \"link\" : \"r_gripper_tool_frame\",\
		        }\
		    }\
	    ],\
	    \"init_info\" : {\
    		\"type\" : \"given_traj\",\
    		\"data\" : [-0.66 -0.04 -0.77 -1.11 -1.82 -1.49 -2.91  0.12  0.98  0.13  3.42]\
    	}\
	}";
	
*/

	request << "{\"basic_info\":{\
\"n_steps\":1,\
\"manip\":\"active\",\
\"start_fixed\":false \
},\
\"costs\":[\
{\
\"type\":\"collision\",\
\"params\":{\"coeffs\" : [10],\
\"dist_pen\":[0.025] \
}\
}\
],\
\"constraints\" :[\
{\
\"type\":\"pose\",\
\"name\":\"final_pose\",\
\"params\":{\"pos_coeffs\":[1,1,1],\
\"rot_coeffs\":[1,1,1],\
\"xyz\":"<< convertDoubleVectortoString(xyz_targ) <<",\
\"wxyz\":"<< convertDoubleVectortoString(quat_targ) <<",\
\"link\": \"r_gripper_tool_frame\" \
}\
}\
],\
\"init_info\":{\
\"type\":\"given_traj\",\
\"data\": [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]\
}\
}";
//"<< convertDoubleVectortoString(DofVals) <<
//{-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1]
//-1.76080454 -0.1797336 -1.99076666 -0.21800647 0.73899571 -0.18675692 2.13973504 0.14755146 0.38328063 -0.48618576 1.33518369
	 //Init val generation:
	 //empty
	 //\"data\" : [-0.66 -0.04 -0.77 -1.11 -1.82 -1.49 -2.91  0.12  0.98  0.13  3.42]\
	 //Currently just using previously randomly genearted numbers from the python file in data in init_info in the json
	 
	 //request["init_info"]["type"] = "given_traj"


std::cerr<<"Converting convertDoubleVectortoString(DofVals) to an array of an array:"<<std::endl;
std::cerr<<convertDoubleVectortoString(DofVals)<<std::endl;
	 // Parsing:
	 Json::Value root;
	 Json::Reader reader;
	 bool parsedSuccess = reader.parse(request,
	                                   root,
	                                   false);

//std::cerr<<convertDoubleVectortoString(DofVals).size()<<std::endl;
//std::cerr<<convertDoubleVectortoString(DofVals)<<std::endl;

	 if(not parsedSuccess)
	 {
		 
		 std::cerr<<"Failed to parse JSON"<<std::endl;
		 std::cerr<<convertDoubleVectortoString(DofVals)<<std::endl
	       <<reader.getFormatedErrorMessages()
	       <<std::endl;
	   //return 1;
	 }

	std::cout<<request<<std::endl;


	trajopt::ProblemConstructionInfo pci(penv);

	//Altering 
	trajopt::InitInfo info;
	//info.type = "given_traj";
	//info.data = {-0.66, -0.04, -0.77, -1.11, -1.82, -1.49, -2.91,  0.12,  0.98,  0.13,  3.42};
	pci.init_info = info;

	pci.fromJson(root);

	trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(pci);

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

	bool success = false;


	OpenRAVE::RaveDestroy();
	return 0;
}


