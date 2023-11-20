#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <filesystem>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif

#include <RVO.h>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;


/*这部分代码用于实现ORCA避障*/
//构建场景
void setupScenario(RVO::RVOSimulator* sim, std::vector<std::vector<float>>& obstacles, std::vector<std::vector<float>>& agent_Pos, std::vector<std::vector<float>>& agent_Goals)
{
#if RVO_SEED_RANDOM_NUMBER_GENERATOR
    std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

    /*这里的参数可以调节模拟的时间步步长，数值越大，模拟越不精准，甚至可能会导致死锁或发生碰撞，
    但是相邻两次模拟的位移越大。时间步参数表示的就是多久调用ORCA计算调整一次速度大小和方向*/
    /* Specify the global time step of the simulation. */
    sim->setTimeStep(1.5f);

    /* Specify the default parameters for agents that are subsequently added. */
    sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 30.0f, 2.0f);
    /*
    参数列表 
    float neighborDist,
    size_t maxNeighbors,
    float timeHorizon,
    float timeHorizonObst,
    float radius,
    float maxSpeed,
    const Vector2 &velocity
    timeHorizon（时间视野）： 增加timeHorizon可能使代理更有长远的预测能力，有助于避免与其他代理的碰撞。
    如果代理在遇到其他代理时卡住，增加时间视野可能有助于更好地规划避让策略。
    timeHorizonObst（障碍物时间视野）： 同样，增加timeHorizonObst可能使代理更早地察觉到障碍物，有助于
    规划更充分的避让策略。如果代理在接近障碍物时出现问题，可以尝试增加这个时间视野。

    用 当前时间步智能体间的安全距离/timeHrizon 可以得到避免碰撞的速度域，进而通过调整现有速度避免碰撞。

    对于SimulationDisplay项目中的舰员来说，radius设置为30较为合适。
    */

    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */

    //获取我们需要定义的智能体个数
    std::size_t elementCount = agent_Pos.size();
    //std::cout << elementCount << std::endl;

    //向模拟器中添加智能体
    for (size_t i = 0; i < elementCount; ++i) {
        float pos_x = agent_Pos[i][0];
        float pos_y = agent_Pos[i][1];
        sim->addAgent(RVO::Vector2(pos_x, pos_y));

        float goal_x = agent_Goals[i][0];
        float goal_y = agent_Goals[i][1];
        goals.push_back(RVO::Vector2(goal_x, goal_y));
    }

    /*
     * Add (polygonal) obstacles, specifying their vertices in counterclockwise
     * order.
     */
    //获取我们需要定义的障碍物个数
    std::size_t obstacleCount = obstacles.size();
    //向模拟器中添加障碍物
    for (size_t i = 0; i < obstacleCount; ++i) {
        std::vector<RVO::Vector2> obs;
        
        float pos1_x = obstacles[i][0];
        float pos1_y = obstacles[i][1];
        
        float pos2_x = obstacles[i][2];
        float pos2_y = obstacles[i][3];
        
        float pos3_x = obstacles[i][4];
        float pos3_y = obstacles[i][5];
        
        float pos4_x = obstacles[i][6];
        float pos4_y = obstacles[i][7];

        obs.push_back((RVO::Vector2(pos1_x, pos1_y)));
        obs.push_back((RVO::Vector2(pos2_x, pos2_y)));
        obs.push_back((RVO::Vector2(pos3_x, pos3_y)));
        obs.push_back((RVO::Vector2(pos4_x, pos4_y)));

        sim->addObstacle(obs);
    }

    /* Process the obstacles so that they are accounted for in the simulation. */
    sim->processObstacles();
}


//显示当前全局时间以及智能体所处的位置
#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator* sim)
{
    /* Output the current global time. */
    std::cout << sim->getGlobalTime();

    /* Output the current position of all the agents. */
    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        std::cout << " " << sim->getAgentPosition(i);
    }

    std::cout << std::endl;
}
#endif


//设置优先速度（即根据所处位置和目的地的位置关系设置速度大小及方向）
void setPreferredVelocities(RVO::RVOSimulator* sim)
{
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
        RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

        //这里对速度大小进行限制，如果目的地和当前位置之间的距离绝对值超过，则会把速度归一化使速度绝对值为1
        if (RVO::absSq(goalVector) > 1.0f) {
            goalVector = RVO::normalize(goalVector);
        }

        sim->setAgentPrefVelocity(i, goalVector);

        /*
         * Perturb a little to avoid deadlocks due to perfect symmetry.
         */
        //对速度大小和方向进行细微的扰动，防止死锁
        float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
        float dist = std::rand() * 0.0001f / RAND_MAX;

        sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
            dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
    }
}


//将当前的位置和速度存入项目文件夹下的 “path/自定义目录” 下，每一个时间步的计算结果保存为一个文件
void pathPointEntering(RVO::RVOSimulator* sim, std::vector<float> agent_ID, std::string folder) {
    
    //拿到当前时间步
    float global_Time = sim->getGlobalTime();
    float timeStep = sim->getTimeStep();
    float step = global_Time / timeStep;
    int step_Counter = int(step);

    //下面的代码用于为时间步数字前面补0使其成为5个数字构成的文件名，同时类型转为字符串
    //这里实际上可以为任意的文件名，6位7位都可以（更改std::setw(6)中的数字即可改变位数），名字也可已任起只要能保证文件按照生成顺序排序即可
    std::string currentFileName;
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << step_Counter;
	ss >> currentFileName;
    //std::cout << typeid(currentFileName).name() << "*" << bb << "*" << std::endl;

    /*构建文件路径*/
    std::string fullPath = "../navigator/" + folder + currentFileName + ".txt";
    //创建文件流对象并打开文件
    std::ofstream outputFile(fullPath);

    //检查文件是否成功打开
    if (outputFile.is_open()) {
        std::cout << "文件已成功创建并打开" << std::endl;
    }
    else {
        std::cerr << "无法打开文件\n" << std::endl;
        return;
    }

    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        //拿到当前agent编号
        int ID = int(agent_ID[i]);
        
        //拿到当前时间步下智能体位置
        RVO::Vector2 pos = sim->getAgentPosition(i);
        float pos_x = pos.x();
        float pos_y = pos.y();

        //拿到当前时间步下智能体速度
        RVO::Vector2 velocity = sim->getAgentVelocity(i);
        float velocity_x = velocity.x();
        float velocity_y = velocity.y();
        //std::cout << typeid(pos).name() << std::endl;
        //std::cout << typeid(pos_x).name() << std::endl;
        //std::cout << typeid(pos_y).name() << std::endl;
        
        //将拿到的信息写入文件
        outputFile << step_Counter + 1 << " " << ID << " " << pos_x << " " << pos_y << " " << velocity_x << " " << velocity_y << std::endl;         

    }
    std::cout << std::endl;
    outputFile.close();
}

//判断是否到达目的地
bool reachedGoal(RVO::RVOSimulator* sim)
{
    /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 20.0f * 20.0f) {
            return false;
        }
    }
    return true;
}


//读取智能体位置目标文件并存入三个容器（，智能体ID容器，位置容器，目标容器）
bool readAgent(std::string filePath, std::vector<float>& agent_ID, std::vector<std::vector<float>>& agent_Pos, std::vector<std::vector<float>>& agent_Goals) {

    // 打开智能体描述文件
    std::ifstream inputFile(filePath);

    // 检查文件是否成功打开
    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件: " << filePath << std::endl;
        return false;  // 退出程序，返回错误代码
    }

    // 逐行读取，智能体位置信息存入 agent_Pos 容器中，智能体目标信息存入 agent_Goals 容器中
    std::vector<float> pos;
    std::vector<float> goal;
    std::vector<float> mix;
    //设置计数器
    int count = 0;
    std::string line;

    //先把ID，位置和目标信息都读入mix容器
    while (std::getline(inputFile, line)) {
        float a = std::stof(line);
        mix.push_back(a);
        count += 1;
    }

    //分别读入agent_Pos 容器和 agent_Goals 容器中
    count /= 5;
    //std::cout << count << std::endl;
    for (int i = 0; i < count; i++) {
        float temp = mix[i * 5];
        agent_ID.push_back(temp);
        temp = mix[i * 5 + 1];
        pos.push_back(temp);
        temp = mix[i * 5 + 2];
        pos.push_back(temp);
        agent_Pos.push_back(pos);
        pos.clear();

        temp = mix[i * 5 + 3];
        goal.push_back(temp);
        temp = mix[i * 5 + 4];
        goal.push_back(temp);
        agent_Goals.push_back(goal);
        goal.clear();
    }

    //检验读取的结果
    /*for (const auto& element : agent_Pos) {
        for (const auto& element1 : element) {
            std::cout << element1 << std::endl;
            std::cout << typeid(element1).name() << std::endl;
        }
        std::cout << "" << std::endl;
    }
    for (const auto& element : agent_Goals) {
        for (const auto& element1 : element) {
            std::cout << element1 << std::endl;
            std::cout << typeid(element1).name() << std::endl;
        }
        std::cout << "" << std::endl;
    }*/

    // 关闭文件
    inputFile.close();

    return true
        ;  // 正常退出

}


//读取障碍物位置文件并存入容器
bool readObstacle(std::string filePath, std::vector<std::vector<float>> &obstacles) {

    // 打开障碍物位置文件
    std::ifstream inputFile(filePath);

    // 检查文件是否成功打开
    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件: " << filePath << std::endl;
        return false;  // 退出程序，返回错误代码
    }

    // 逐行读取，障碍物存入 obstacles 容器中
    std::vector<float> obstacle_Pos;
    //设置计数器
    int count = 0;
    std::string line;

    while (std::getline(inputFile, line)) {
        //std::cout << line << std::endl;
        float dot_Pos = std::stof(line);
        obstacle_Pos.push_back(dot_Pos);
        count += 1;

        if (count % 8 == 0) {
            obstacles.push_back(obstacle_Pos);
            obstacle_Pos.clear();
        }

    }

    //检验读取的结果
    /*for (const auto& element : obstacles) {
        for (const auto& element1 : element) {
            std::cout << element1 << std::endl;
            std::cout << typeid(element1).name() << std::endl;
        }
        std::cout << "--------------------------" << std::endl;
    }*/

    // 关闭文件
    inputFile.close();

    return true
;  // 正常退出

}

int main() {

    std::vector<float> agent_ID;
    std::vector<std::vector<float>> obstacles;
    std::vector<std::vector<float>> agent_Pos;
    std::vector<std::vector<float>> agent_Goals;

    /* folder 为存放路径文件的目录名，该目录位于项目下的 navigator 目录中
       注意一定的要建好目录再运行
       obstacles_loadingPath 为存放障碍物位置的文件，更改这项则可以改变障碍物读取的路径
       agents_loadingPath 为存放障智能体位置和目的地的文件，更改这项则可以改变障碍物读取的路径*/
    std::string folder = "03_fourAgents_twoObstacles/";
    std::string obstacles_loadingPath = "../config/obstacles.txt";
    std::string agents_loadingPath = "../config/agent.txt";
    readObstacle(obstacles_loadingPath, obstacles);
    readAgent(agents_loadingPath,agent_ID, agent_Pos, agent_Goals);

    /* Create a new simulator instance. */
    RVO::RVOSimulator* sim = new RVO::RVOSimulator();

    /* Set up the scenario. */
    setupScenario(sim, obstacles, agent_Pos, agent_Goals);

    do {
    #if RVO_OUTPUT_TIME_AND_POSITIONS
        updateVisualization(sim);
    #endif
        pathPointEntering(sim, agent_ID, folder);
        setPreferredVelocities(sim);
        sim->doStep();
    } while (!reachedGoal(sim));

    delete sim;

    return 0;

}