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


/*�ⲿ�ִ�������ʵ��ORCA����*/
//��������
void setupScenario(RVO::RVOSimulator* sim, std::vector<std::vector<float>>& obstacles, std::vector<std::vector<float>>& agent_Pos, std::vector<std::vector<float>>& agent_Goals)
{
#if RVO_SEED_RANDOM_NUMBER_GENERATOR
    std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

    /*����Ĳ������Ե���ģ���ʱ�䲽��������ֵԽ��ģ��Խ����׼���������ܻᵼ������������ײ��
    ������������ģ���λ��Խ��ʱ�䲽������ʾ�ľ��Ƕ�õ���ORCA�������һ���ٶȴ�С�ͷ���*/
    /* Specify the global time step of the simulation. */
    sim->setTimeStep(1.5f);

    /* Specify the default parameters for agents that are subsequently added. */
    sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 30.0f, 2.0f);
    /*
    �����б� 
    float neighborDist,
    size_t maxNeighbors,
    float timeHorizon,
    float timeHorizonObst,
    float radius,
    float maxSpeed,
    const Vector2 &velocity
    timeHorizon��ʱ����Ұ���� ����timeHorizon����ʹ������г�Զ��Ԥ�������������ڱ����������������ײ��
    ���������������������ʱ��ס������ʱ����Ұ���������ڸ��õع滮���ò��ԡ�
    timeHorizonObst���ϰ���ʱ����Ұ���� ͬ��������timeHorizonObst����ʹ�������ز�����ϰ��������
    �滮����ֵı��ò��ԡ���������ڽӽ��ϰ���ʱ�������⣬���Գ����������ʱ����Ұ��

    �� ��ǰʱ�䲽�������İ�ȫ����/timeHrizon ���Եõ�������ײ���ٶ��򣬽���ͨ�����������ٶȱ�����ײ��

    ����SimulationDisplay��Ŀ�еĽ�Ա��˵��radius����Ϊ30��Ϊ���ʡ�
    */

    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */

    //��ȡ������Ҫ��������������
    std::size_t elementCount = agent_Pos.size();
    //std::cout << elementCount << std::endl;

    //��ģ���������������
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
    //��ȡ������Ҫ������ϰ������
    std::size_t obstacleCount = obstacles.size();
    //��ģ����������ϰ���
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


//��ʾ��ǰȫ��ʱ���Լ�������������λ��
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


//���������ٶȣ�����������λ�ú�Ŀ�ĵص�λ�ù�ϵ�����ٶȴ�С������
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

        //������ٶȴ�С�������ƣ����Ŀ�ĵغ͵�ǰλ��֮��ľ������ֵ�����������ٶȹ�һ��ʹ�ٶȾ���ֵΪ1
        if (RVO::absSq(goalVector) > 1.0f) {
            goalVector = RVO::normalize(goalVector);
        }

        sim->setAgentPrefVelocity(i, goalVector);

        /*
         * Perturb a little to avoid deadlocks due to perfect symmetry.
         */
        //���ٶȴ�С�ͷ������ϸ΢���Ŷ�����ֹ����
        float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
        float dist = std::rand() * 0.0001f / RAND_MAX;

        sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
            dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
    }
}


//����ǰ��λ�ú��ٶȴ�����Ŀ�ļ����µ� ��path/�Զ���Ŀ¼�� �£�ÿһ��ʱ�䲽�ļ���������Ϊһ���ļ�
void pathPointEntering(RVO::RVOSimulator* sim, std::vector<float> agent_ID, std::string folder) {
    
    //�õ���ǰʱ�䲽
    float global_Time = sim->getGlobalTime();
    float timeStep = sim->getTimeStep();
    float step = global_Time / timeStep;
    int step_Counter = int(step);

    //����Ĵ�������Ϊʱ�䲽����ǰ�油0ʹ���Ϊ5�����ֹ��ɵ��ļ�����ͬʱ����תΪ�ַ���
    //����ʵ���Ͽ���Ϊ������ļ�����6λ7λ�����ԣ�����std::setw(6)�е����ּ��ɸı�λ����������Ҳ��������ֻҪ�ܱ�֤�ļ���������˳�����򼴿�
    std::string currentFileName;
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << step_Counter;
	ss >> currentFileName;
    //std::cout << typeid(currentFileName).name() << "*" << bb << "*" << std::endl;

    /*�����ļ�·��*/
    std::string fullPath = "../navigator/" + folder + currentFileName + ".txt";
    //�����ļ������󲢴��ļ�
    std::ofstream outputFile(fullPath);

    //����ļ��Ƿ�ɹ���
    if (outputFile.is_open()) {
        std::cout << "�ļ��ѳɹ���������" << std::endl;
    }
    else {
        std::cerr << "�޷����ļ�\n" << std::endl;
        return;
    }

    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        //�õ���ǰagent���
        int ID = int(agent_ID[i]);
        
        //�õ���ǰʱ�䲽��������λ��
        RVO::Vector2 pos = sim->getAgentPosition(i);
        float pos_x = pos.x();
        float pos_y = pos.y();

        //�õ���ǰʱ�䲽���������ٶ�
        RVO::Vector2 velocity = sim->getAgentVelocity(i);
        float velocity_x = velocity.x();
        float velocity_y = velocity.y();
        //std::cout << typeid(pos).name() << std::endl;
        //std::cout << typeid(pos_x).name() << std::endl;
        //std::cout << typeid(pos_y).name() << std::endl;
        
        //���õ�����Ϣд���ļ�
        outputFile << step_Counter + 1 << " " << ID << " " << pos_x << " " << pos_y << " " << velocity_x << " " << velocity_y << std::endl;         

    }
    std::cout << std::endl;
    outputFile.close();
}

//�ж��Ƿ񵽴�Ŀ�ĵ�
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


//��ȡ������λ��Ŀ���ļ�������������������������ID������λ��������Ŀ��������
bool readAgent(std::string filePath, std::vector<float>& agent_ID, std::vector<std::vector<float>>& agent_Pos, std::vector<std::vector<float>>& agent_Goals) {

    // �������������ļ�
    std::ifstream inputFile(filePath);

    // ����ļ��Ƿ�ɹ���
    if (!inputFile.is_open()) {
        std::cerr << "�޷����ļ�: " << filePath << std::endl;
        return false;  // �˳����򣬷��ش������
    }

    // ���ж�ȡ��������λ����Ϣ���� agent_Pos �����У�������Ŀ����Ϣ���� agent_Goals ������
    std::vector<float> pos;
    std::vector<float> goal;
    std::vector<float> mix;
    //���ü�����
    int count = 0;
    std::string line;

    //�Ȱ�ID��λ�ú�Ŀ����Ϣ������mix����
    while (std::getline(inputFile, line)) {
        float a = std::stof(line);
        mix.push_back(a);
        count += 1;
    }

    //�ֱ����agent_Pos ������ agent_Goals ������
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

    //�����ȡ�Ľ��
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

    // �ر��ļ�
    inputFile.close();

    return true
        ;  // �����˳�

}


//��ȡ�ϰ���λ���ļ�����������
bool readObstacle(std::string filePath, std::vector<std::vector<float>> &obstacles) {

    // ���ϰ���λ���ļ�
    std::ifstream inputFile(filePath);

    // ����ļ��Ƿ�ɹ���
    if (!inputFile.is_open()) {
        std::cerr << "�޷����ļ�: " << filePath << std::endl;
        return false;  // �˳����򣬷��ش������
    }

    // ���ж�ȡ���ϰ������ obstacles ������
    std::vector<float> obstacle_Pos;
    //���ü�����
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

    //�����ȡ�Ľ��
    /*for (const auto& element : obstacles) {
        for (const auto& element1 : element) {
            std::cout << element1 << std::endl;
            std::cout << typeid(element1).name() << std::endl;
        }
        std::cout << "--------------------------" << std::endl;
    }*/

    // �ر��ļ�
    inputFile.close();

    return true
;  // �����˳�

}

int main() {

    std::vector<float> agent_ID;
    std::vector<std::vector<float>> obstacles;
    std::vector<std::vector<float>> agent_Pos;
    std::vector<std::vector<float>> agent_Goals;

    /* folder Ϊ���·���ļ���Ŀ¼������Ŀ¼λ����Ŀ�µ� navigator Ŀ¼��
       ע��һ����Ҫ����Ŀ¼������
       obstacles_loadingPath Ϊ����ϰ���λ�õ��ļ���������������Ըı��ϰ����ȡ��·��
       agents_loadingPath Ϊ�����������λ�ú�Ŀ�ĵص��ļ���������������Ըı��ϰ����ȡ��·��*/
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