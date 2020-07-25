#include <actionlib_demo/DoDishesAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

class ActionlibDemoServer
{
	typedef actionlib::SimpleActionServer<actionlib_demo::DoDishesAction> Server;
private:
	Server server_;
	void execute(const actionlib_demo::DoDishesGoalConstPtr& goal)  // Note: "Action" is not appended to DoDishes here
	{
		ROS_INFO("Server Received a goal");
		
		ros::NodeHandle nh;
		int i =1;
		while(nh.ok())
		{
			if(server_.isPreemptRequested())
			{
				ROS_INFO("isPreemptRequested");
				if(server_.isNewGoalAvailable())
				{
					ROS_INFO("isNewGoalAvailable");
					actionlib_demo::DoDishesGoal new_goal = *server_.acceptNewGoal();
					// Do lots of awesome groundbreaking robot stuff here
					i = 0;
				}
				else
				{
					if(server_.isActive())
						server_.setSucceeded();
					return;
				}
			}
			
			if(!server_.isActive())break;

			ROS_INFO("Cleanning the dish : %d", i);

			actionlib_demo::DoDishesFeedback feedback;
			feedback.percent_complete = i/20.0;
			server_.publishFeedback(feedback);
			
			if(i++ == 20)
			{
				actionlib_demo::DoDishesResult result;
				result.total_dishes_cleaned = 20;
				if(server_.isActive())
					server_.setSucceeded(result);
			}
				
			ros::Duration(0.5).sleep();
		}
		
	}
public:
	ActionlibDemoServer(ros::NodeHandle nh):
		server_(nh, "do_dishes", boost::bind(&ActionlibDemoServer::execute, this, _1), false)
	{
	
	}
	bool init()
	{
		server_.start();
		return true;
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "do_dishes_server");
	ros::NodeHandle nh;
	ActionlibDemoServer app(nh);
	app.init();
	ros::spin();
	return 0;
}


