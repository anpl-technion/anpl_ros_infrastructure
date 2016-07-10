/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna, Luis G. Torres */

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <boost/graph/astar_search.hpp>
#include <iostream>

// We will use Pose3 variables (x, y, z) to represent the robot positions
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;
using namespace gtsam;

void omplPathToGtsamGraph(og::PathGeometric path);

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

void planWithSimpleSetup(void)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE3StateSpace());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setX(-9);
    start->setY(0);
    start->setZ(0);
    start->rotation().setIdentity(); // Set the state to identity – no rotation.
    // start.random();

    // create a random goal state
    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setX(9);
    goal->setY(0);
    goal->setZ(0);
    goal->rotation().setAxisAngle(0, 0, 0, 1); // Set the quaternion from axis-angle representation.

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

	
    // change planner
//    cout << "==== change planner ====" << endl;
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

	// print params
//    cout << "==== change range ====" << endl;
    double range = 10;
    ompl::base::ParamSet& params = planner->params();
    if (params.hasParam(std::string("range"))) {
        params.setParam(std::string("range"), boost::lexical_cast<std::string>(range));
    }
//	cout << "==== print params ====" << endl;
    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();
	//ss.getPlanner()->printSettings(cout);
	//exit(0);

    // attempt to find an exact solution within five seconds
    if (ss.solve(5.0) == ob::PlannerStatus::EXACT_SOLUTION)
    {
        og::PathGeometric slnPath = ss.getSolutionPath();

        std::cout << std::endl;
        std::cout << "Found solution with " << slnPath.getStateCount() << " states and length " << slnPath.length() << std::endl;
        // print the path to screen
        //slnPath.print(std::cout);

        std::cout << "Writing PlannerData to file './myPlannerData'" << std::endl;
        ob::PlannerData data(ss.getSpaceInformation());
        ss.getPlannerData(data);

        ob::PlannerDataStorage dataStorage;
        dataStorage.store(data, "myPlannerData");

        slnPath.print(cout);
        omplPathToGtsamGraph(slnPath);
    }
    else
        std::cout << "No solution found" << std::endl;
}



// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                           vertex_type_t>::type& plannerDataVertices)
{
    return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}


void readPlannerData(void)
{
    std::cout << std::endl;
    std::cout << "Reading PlannerData from './myPlannerData'" << std::endl;

    // Recreating the space information from the stored planner data instance
    ob::StateSpacePtr space(new ob::SE3StateSpace());
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ob::PlannerDataStorage dataStorage;
    ob::PlannerData data(si);

    // Loading an instance of PlannerData from disk.
    dataStorage.load("myPlannerData", data);

    // Re-extract the shortest path from the loaded planner data
    if (data.numStartVertices() > 0 && data.numGoalVertices() > 0)
    {
        // Create an optimization objective for optimizing path length in A*
        ob::PathLengthOptimizationObjective opt(si);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        data.computeEdgeWeights(opt);

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

        // Now we can apply any Boost.Graph algorithm.  How about A*!

        // create a predecessor map to store A* results in
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

        // Retieve a property map with the PlannerDataVertex object pointers for quick lookup
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

        // Run A* search over our planner data
        ob::GoalState goal(si);
        goal.setState(data.getGoalVertex(0).getState());
        ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
        boost::astar_search(graph, start,
                            boost::bind(&distanceHeuristic, _1, &goal, &opt, vertices),
                            boost::predecessor_map(prev).
                            distance_compare(boost::bind(&ob::OptimizationObjective::
                                                         isCostBetterThan, &opt, _1, _2)).
                            distance_combine(boost::bind(&ob::OptimizationObjective::
                                                         combineCosts, &opt, _1, _2)).
                            distance_inf(opt.infiniteCost()).
                            distance_zero(opt.identityCost()));

        // Extracting the path
        og::PathGeometric path(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
             prev[pos] != pos;
             pos = prev[pos])
        {
            path.append(vertices[pos]->getState());
        }
        path.append(vertices[start]->getState());
        path.reverse();

        // print the path to screen
        //path.print(std::cout);
        std::cout << "Found stored solution with " << path.getStateCount() << " states and length " << path.length() << std::endl;
    }
}

gtsam::Pose3* omplStateToGtsamPose3(ob::SE3StateSpace::StateType* state){
	gtsam::Point3 point3(state->getX(),state->getY(),state->getZ());
	const ob::SO3StateSpace::StateType& rotation = state->rotation();
	gtsam::Quaternion quaternion(rotation.w,rotation.x,rotation.y,rotation.z);
	gtsam::Rot3 rot3(quaternion);
	return new gtsam::Pose3(rot3,point3);
}

void omplPathToGtsamGraph(og::PathGeometric path) {
	// Create an empty nonlinear factor graph
	gtsam::NonlinearFactorGraph graph;

	// Add a prior on the first pose, setting it to the origin
	// A prior factor consists of a mean and a noise model (covariance matrix)
	std::vector<ob::State*>& states = path.getStates();


	gtsam::Pose3* startPose = omplStateToGtsamPose3(states.front()->as<ob::SE3StateSpace::StateType>());
	// prior at origin

	// containers
	vector<Symbol*> 	symbols;
	vector<Pose3*>		poses;
	Values				values;

	Symbol* firstSymbol = new Symbol('X',0);
	symbols.push_back(firstSymbol);
	values.insert(firstSymbol->key(),*startPose);
	poses.push_back(startPose);

	gtsam::noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4));
	gtsam::PriorFactor<gtsam::Pose3> priorFactor(firstSymbol->key(), *startPose, priorNoise);
	graph.add(priorFactor);
	long state_index = 0;

	for (vector<ob::State*>::iterator it = states.begin()+1; it != states.end(); ++it)
	{
		state_index++;
		Symbol* newSymbol = new Symbol('x',state_index);
		gtsam::Pose3* newPose = omplStateToGtsamPose3((*it)->as<ob::SE3StateSpace::StateType>());
		Symbol* previousSymbol 	= symbols.back();
		Pose3* previousPose	= poses.back();
		Pose3 control			= previousPose->between(*newPose);
		BetweenFactor<Pose3> factor = BetweenFactor<Pose3>(previousSymbol->key(), newSymbol->key(),control,priorNoise);
		graph.add(factor);
		symbols.push_back(newSymbol);
		poses.push_back(newPose);
		values.insert(newSymbol->key(),*newPose);
	}

	  // optimize using Levenberg-Marquardt optimization
	  Values result = LevenbergMarquardtOptimizer(graph, values).optimize();
	  result.print("Final Result:\n");

	  // Calculate and print marginal covariances for all variables
	  cout.precision(2);
	  Marginals marginals(graph, result);
	  // show how the covariance evolves for first 3 poses
	  for (int i = 0; i < 3; ++i) {
		  cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(symbols.at(i)->key()) << endl;
	  }

}

int main(int, char **)
{
    // Plan and save all of the planner data to disk
    planWithSimpleSetup();

    // Read in the saved planner data and extract the solution path
    //readPlannerData();

    return 0;
}
