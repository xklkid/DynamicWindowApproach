#include <iostream>
#include "Constant.h"
#include <numeric>

using namespace std;

vector<float> DynamicWindowApproach(RobotState rState, int obs[][2], int target[]);
RobotState Motion(RobotState curState, float velocity, float omega);
vector<float> CreateDW(RobotState state);
vector<RobotState> GenerateTraj(RobotState initState, float vel, float ome);
float CalcHeading(RobotState rState, int goal[]);
float CalcClearance(RobotState rState, int obs[][2]);
float CalcBreakingDist(float velo);

void main()
{
	// 初始化机器人当前的参数
	RobotState currentState = { 0, 0, M_PI / 2, 0, 0 };
	int goal[2] = { GOAL_X, GOAL_Y };
	vector<RobotState> path;
	cout << "Begin!" << endl;

	while (1)
	{
		// 到达目标点退出循环
		if (currentState.xPosition == goal[0] && currentState.yPosition == goal[1])
		{
			cout << "Reach the Goal!" << endl;
			break;
		}
		vector<float> selectedVelocity = DynamicWindowApproach(currentState, obstacle, goal);

		// 机器人移动
		currentState = Motion(currentState, selectedVelocity[0], selectedVelocity[1]);

		path.push_back(currentState);

		//cout << currentState.xPosition << " " << currentState.yPosition << endl;
	}

	for (vector<RobotState>::iterator i = path.begin(); i < path.end(); i++)
	{
		cout << i->yPosition << " " << i->xPosition << endl;
	}

}

vector<float> DynamicWindowApproach(RobotState rState, int obs[][2], int target[])
{
	// 0:minVelocity, 1:maxVelocity, 2:minOmega, 3:maxOmega
	vector<float> velocityAndOmegaRange = CreateDW(rState);
	vector<EvaluationPara>evalParas;
	float sumHeading = 0;
	float sumClearance = 0;
	float sumVelocity = 0;

	for (double v = velocityAndOmegaRange[0]; v < velocityAndOmegaRange[1]; v += SAMPLING_VELOCITY)
	{
		for (double w = velocityAndOmegaRange[2]; w < velocityAndOmegaRange[3]; w += SAMPLING_OMEGA)
		{
			vector<RobotState> trajectories = GenerateTraj(rState, v, w);

			//评价参数
			EvaluationPara tempEvalPara;
			float tempClearance = CalcClearance(trajectories.back(), obstacle);
			float stopDist = CalcBreakingDist(v);
			if (tempClearance > stopDist)
			{
				tempEvalPara.heading = CalcHeading(trajectories.back(), target);
				tempEvalPara.clearance = tempClearance;
				tempEvalPara.velocity = abs(v);
				tempEvalPara.v = v;
				tempEvalPara.w = w;

				sumHeading = sumHeading + tempEvalPara.heading;
				sumClearance = sumHeading + tempEvalPara.clearance;
				sumVelocity = sumVelocity + tempEvalPara.velocity;

				evalParas.push_back(tempEvalPara);
			}
		}
	}

	//平滑评价参数并选择最优速度
	float selectedVelocity = 0;
	float selectedOmega = 0;
	float G = 0;
	for (vector<EvaluationPara>::iterator i = evalParas.begin(); i < evalParas.end(); i++)
	{
		float smoothHeading = i->heading / sumHeading;
		float smoothClearance = i->clearance / sumClearance;
		float smoothVelocity = i->velocity / sumVelocity;

		float tempG = WEIGHT_HEADING*smoothHeading + WEIGHT_CLEARANCE*smoothClearance + WEIGHT_VELOCITY*smoothVelocity;

		if (tempG > G)
		{
			G = tempG;
			selectedVelocity = i->v;
			selectedOmega = i->w;
		}
	}

	vector<float> selVelocity(2);
	selVelocity[0] = selectedVelocity;
	selVelocity[1] = selectedOmega;

	return selVelocity;
}

vector<float> CreateDW(RobotState curState)
{
	vector<float> dw(4);
	float tmpMinVelocity = curState.velocity - MAX_ACCELERATE*DT;
	float tmpMaxVelocity = curState.velocity + MAX_ACCELERATE*DT;
	float tmpMinOmega = curState.omega - MAX_ACCOMEGA*DT;
	float tmpMaxOmega = curState.omega + MAX_ACCOMEGA*DT;

	//dw[0] = tmpMinVelocity > MIN_VELOCITY ? tmpMinVelocity : MIN_VELOCITY;
	dw[0] = 0;
	dw[1] = tmpMaxVelocity < MAX_VELOCITY ? tmpMaxVelocity : MAX_VELOCITY;
	dw[2] = tmpMinOmega;
	dw[3] = tmpMaxOmega < MAX_OMEGA ? tmpMaxOmega : MAX_OMEGA;

	return dw;
}

RobotState Motion(RobotState curState, float velocity, float omega)
{
	RobotState afterMoveState;

	//if (omega != 0)
	//{
	//	afterMoveState.xPosition = curState.xPosition + velocity / omega*sin(curState.orientation)
	//		- velocity / omega*sin(curState.orientation + omega*DT);

	//	afterMoveState.yPosition = curState.yPosition - velocity / omega*cos(curState.orientation)
	//		- velocity / omega*cos(curState.orientation + omega*DT);
	//}
	//else
	//{
	//	afterMoveState.xPosition = curState.xPosition + velocity*cos(curState.orientation)*DT;

	//	afterMoveState.yPosition = curState.yPosition + velocity*sin(curState.orientation)*DT;
	//}

	afterMoveState.xPosition = curState.xPosition + velocity*DT*cos(curState.orientation);
	afterMoveState.yPosition = curState.yPosition + velocity*DT*sin(curState.orientation);

	afterMoveState.orientation = curState.orientation + omega * DT;
	afterMoveState.velocity = velocity;
	afterMoveState.omega = omega;	

	return afterMoveState;
}

vector<RobotState> GenerateTraj(RobotState initState, float vel, float ome)
{
	RobotState tempState = initState;
	vector<RobotState> trajectories;
	float time = 0;
	trajectories.push_back(initState);
	while (time < PREDICT_TIME)
	{
		tempState = Motion(tempState, vel, ome);
		trajectories.push_back(tempState);
		time += DT;
	}

	return trajectories;
}

float CalcHeading(RobotState rState, int goal[])
{
	float heading;

	float dy = goal[1] - rState.yPosition;
	float dx = goal[0] - rState.xPosition;

	float goalTheta = atan2(dy, dx);
	float targetTheta;
	if (goalTheta > rState.orientation)
	{
		targetTheta = goalTheta - rState.orientation;
	}
	else
	{
		targetTheta = rState.orientation - goalTheta;
	}

	heading = 180 - targetTheta / M_PI * 180;

	return heading;
}

float CalcClearance(RobotState rState, int obs[][2])
{
	float dist = 100;
	float distTemp;
	for (int i = 0; i < 18; i++)
	{
		float dx = rState.xPosition - obs[i][0];
		float dy = rState.yPosition - obs[i][1];
		distTemp = sqrt(pow(dx,2) + pow(dy,2)) - ROBOT_RADIUS;

		if (dist > distTemp)
		{
			dist = distTemp;
		}
	}

	if (dist >= 2 * ROBOT_RADIUS)
	{
		dist = 2 * ROBOT_RADIUS;
	}

	return dist;
}

float CalcBreakingDist(float velo)
{
	float stopDist = 0;
	while (velo > 0)
	{
		stopDist = stopDist + velo*DT;
		velo = velo - MAX_ACCELERATE*DT;
	}

	return stopDist;
}

