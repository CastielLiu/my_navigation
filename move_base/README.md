1. void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal) 
订阅外部目标点请求消息并打包为actionlib发布

2. void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
actionlib回调函数，接收目标点请求信息
正在执行当前目标时可被新目标打断，重新规划路径进行控制

2.1 void MoveBase::planThread()　
路径规划线程，新目标点将唤醒路径规划线程进行路径规划

2.1.1  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
全局路径规划

2.2 bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
局部路径规划，规划期望的运行速度和角速度

