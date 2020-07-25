#include <actionlib_demo/DoDishesAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>


class ActionlibDemoClient
{
	typedef actionlib::SimpleActionClient<actionlib_demo::DoDishesAction> Client;
private:
	Client client_;
	void DoneCb(const actionlib::SimpleClientGoalState& state, const actionlib_demo::DoDishesResultConstPtr& result) 
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Toal dish cleaned: %i", result->total_dishes_cleaned);
		ros::shutdown();
	}

	// 当目标激活的时候，会调用一次
	void ActiveCb() 
	{
		ROS_INFO("Goal just went active");
	}

	// 接收服务器的反馈信息
	void FeedbackCb(const actionlib_demo::DoDishesFeedbackConstPtr& feedback) 
	{
		ROS_INFO("Got Feedback Complete Rate: %f", feedback->percent_complete);
	}
	
public:
	ActionlibDemoClient(const std::string& client_name, bool flag = true) :
		client_(client_name, flag)
	{
	}
	void start() 
	{
		//等待服务器初始化完成
		client_.waitForServer();
		//定义要做的目标
		actionlib_demo::DoDishesGoal goal;
		goal.dishwasher_id = 1;
		//发送目标至服务器
		client_.sendGoal(goal,
		boost::bind(&ActionlibDemoClient::DoneCb, this, _1, _2),
		boost::bind(&ActionlibDemoClient::ActiveCb, this),
		boost::bind(&ActionlibDemoClient::FeedbackCb, this, _1));
		//等待结果
		client_.waitForResult(ros::Duration(15.0));

		//根据返回结果，做相应的处理
		if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			printf("Yay! The dishes are now clean");
		else
		{
			ROS_INFO("Cancel Goal!");
			client_.cancelAllGoals();
		}

		printf("Current State: %s\n", client_.getState().toString().c_str());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "do_dishes_client");
	ActionlibDemoClient app("do_dishes");
	app.start();
	ros::spin();
	return 0;
}
