#include <iostream>
#include <vector>

using namespace std;

#define M_PI 3.1415927
#define MAX_VELOCITY 1.0						//弧形轨迹：最大速度
#define MIN_VELOCITY 0							//弧形轨迹：最小速度
#define MAX_OMEGA 20.0 / 180.0 * M_PI			//弧形轨迹：最大角速度
#define MIN_OMEGA 0								//弧形轨迹：最小角速度
#define MAX_ACCELERATE 0.2						//动态窗口：最大加速度
#define MAX_ACCOMEGA 50.0 / 180.0 * M_PI		//动态窗口：最大角加速度
#define SAMPLING_VELOCITY 0.01					//速度采样间隔
#define SAMPLING_OMEGA 1 / 180.0 * M_PI			//角速度采样间隔
#define DT 0.1									//采样时间间隔
#define PREDICT_TIME 3.0						//预测时间
#define WEIGHT_HEADING 0.05						//HEADING权重
#define WEIGHT_CLEARANCE 0.2					//CLEARANCE权重
#define WEIGHT_VELOCITY 0.1						//VELOCITY权重
#define GOAL_X 10								//目标横坐标
#define GOAL_Y 10								//目标纵坐标
#define ROBOT_RADIUS 0.5						//机器人半径


struct RobotState
{
	// x坐标，y坐标，机器朝向，速度，角速度
	float xPosition, yPosition, orientation, velocity, omega;
};

// 障碍物
int obstacle[18][2] = { { 0, 2 },
							{ 4, 2 },
							{ 4, 4 },
							{ 5, 4 },
							{ 5, 5 },
							{ 5, 6 },
							{ 5, 9 },
							{ 8, 8 },
							{ 8, 9 },
							{ 7, 9 },
							{ 6, 5 },
							{ 6, 3 },
							{ 6, 8 },
							{ 6, 7 },
							{ 7, 4 },
							{ 9, 8 },
							{ 9, 11 },
							{ 9, 6 } };

struct EvaluationPara
{
	float heading, clearance, velocity, v, w;
};