# ORCA_For_Crew
## V0.0
### 在项目下的config目录中，包含有两个文件  
第一个文件agent.txt中存储了智能体的位置和目的地，4行一组，前两行为位置，后两行为目的地。  
第二个文件obstacles.txt中则是存储了静态障碍物的位置信息。在RVO和ORCA中静态障碍物被视为多边形，由一组点连接而成。本实验中我们使用矩形作为障碍物，因而参数为8行一组，分别代表障碍物的4个顶点的坐标。注意要逆时针标注（注：若要与SimulationDispay中的静态横向飞机障碍物相匹配，我们要令障碍物的x轴方	向坐标差值为2000，y轴方向坐标差值为1400）
### 注意在设置目的地和障碍物时，一定要合理，可以事先在方格纸上模拟一下。ORCA可能无法处理及其复杂的躲避障碍物情况。尤其是静态障碍物
### 修改文件读取路径
####在项目 navigator.cpp 文件下的 main 方法中，有以下三个参数：
folder 为存放路径文件的目录名，该目录位于项目下的 navigator 目录中  
obstacles_loadingPath 为存放障碍物位置的文件，更改这项则可以改变障碍物读取的路径  
agents_loadingPath 为存放障智能体位置和目的地的文件，更改这项则可以改变障碍物读取的路径
### 另外说下VS中使用开源代码的方法
首先要在解决方案栏中项目名处右键，打开属性，选择C/C++，选择常规，在右边的附加包含目录中加入头文件所在的目录，告诉编译器在哪里找头文件。
第二部要把开源代码中对于这些头文件进行实现的CPP文件放入源文件目录下，与自己编写的代码文件放在同一个位置上。
***
## V1.0
### 本版本的重大改进
利用ORCA库提供的带有参数列表的addAgent方法对不同的智能体添加不同的碰撞半径(radius)  
提供的API如下  
```
size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
{
	Agent *agent = new Agent(this);

	agent->position_ = position;
	agent->maxNeighbors_ = maxNeighbors;
	agent->maxSpeed_ = maxSpeed;
	agent->neighborDist_ = neighborDist;
	agent->radius_ = radius;
	agent->timeHorizon_ = timeHorizon;
	agent->timeHorizonObst_ = timeHorizonObst;
	agent->velocity_ = velocity;

	agent->id_ = agents_.size();

	agents_.push_back(agent);

	return agents_.size() - 1;
}
```
#### 我们为不同的智能体指定了不同的半径
飞机（喷气式战斗机）：1170.0f  
小车：365.0f  
人员：30.0f
#### 我们如何分辨不同的智能体呢？
根据ID字段值分辨智能体类型：  
ID小于0为飞机，大于等于0小于1000为人员，大于等于1000为车辆
### 经过V0.0的经验总结，我们需要对下列的参数着重注意，它们可能会影响最终的效果
```
		/**
		 * \brief      Constructs a simulator instance and sets the default
		 *             properties for any new agent that is added.
		 * \param      timeStep        The time step of the simulation.
		 *                             Must be positive.
		 * \param      neighborDist    The default maximum distance (center point
		 *                             to center point) to other agents a new agent
		 *                             takes into account in the navigation. The
		 *                             larger this number, the longer he running
		 *                             time of the simulation. If the number is too
		 *                             low, the simulation will not be safe. Must be
		 *                             non-negative.
		 * \param      maxNeighbors    The default maximum number of other agents a
		 *                             new agent takes into account in the
		 *                             navigation. The larger this number, the
		 *                             longer the running time of the simulation.
		 *                             If the number is too low, the simulation
		 *                             will not be safe.
		 * \param      timeHorizon     The default minimal amount of time for which
		 *                             a new agent's velocities that are computed
		 *                             by the simulation are safe with respect to
		 *                             other agents. The larger this number, the
		 *                             sooner an agent will respond to the presence
		 *                             of other agents, but the less freedom the
		 *                             agent has in choosing its velocities.
		 *                             Must be positive.
		 * \param      timeHorizonObst The default minimal amount of time for which
		 *                             a new agent's velocities that are computed
		 *                             by the simulation are safe with respect to
		 *                             obstacles. The larger this number, the
		 *                             sooner an agent will respond to the presence
		 *                             of obstacles, but the less freedom the agent
		 *                             has in choosing its velocities.
		 *                             Must be positive.
		 * \param      radius          The default radius of a new agent.
		 *                             Must be non-negative.
		 * \param      maxSpeed        The default maximum speed of a new agent.
		 *                             Must be non-negative.
		 * \param      velocity        The default initial two-dimensional linear
		 *                             velocity of a new agent (optional).
		 */
		RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors,
					 float timeHorizon, float timeHorizonObst, float radius,
					 float maxSpeed, const Vector2 &velocity = Vector2());
```
#### neighborDist，radius作为RVOSimulator和addAgent共用的参数，可以说是我们进行异质仿真的关键参数
##### neighborDist
对于该参数的理解如下：  
neighborDist 参数是指新代理在导航中考虑其他代理时所使用的默认最大距离。更具体地说，它表示新代理在模拟中对其他代理感知的距离。这里的距离是指代理中心点之间的距离。  
**如果一个智能体半径很大，例如500，而neighborDist这个参数设置的很小，只有50，那可能已经发生了碰撞也没有识别到这个大智能体是自己的邻居从而采取避碰**  
让我们来详细解释这个参数：  
作用： 当一个新的代理（或者机器人、运动体等）在模拟中进行导航时，它需要感知周围其他代理的存在以做出合适的决策，避免碰撞等情况。neighborDist 就是用来定义代理在导航时考虑其他代理的最大距离。  
单位： 距离的单位取决于具体的模拟系统，通常是以长度单位（比如米）来表示。  
模拟运行时间： 当 neighborDist 值较大时，代理需要考虑更远处的其他代理，这可能导致模拟的运行时间较长。这是因为在计算代理的行为时，需要考虑更多的邻近代理，增加了计算的复杂性和时间开销。  
模拟的安全性： 但如果 neighborDist 的值太低，代理可能无法及时感知到远处的其他代理，从而增加了碰撞的风险，模拟可能不安全。合适的 neighborDist 值应当兼顾模拟运行时间和模拟的安全性。  
非负数要求： neighborDist 必须是非负数，因为距离不能是负数。如果设为负数，可能导致意外行为或错误的模拟结果。
##### radius
这个参数就很好理解了，对不同的智能体采取不同的半径设置即可。