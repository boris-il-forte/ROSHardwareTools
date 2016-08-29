#include <ros/ros.h>
#include <std_msgs/Float32.h>


float current = 0;
float alpha = 0.99;

void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
	current = alpha*current + (1.0-alpha)*msg->data;	
	std::cout << current << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_smoother");


  ros::NodeHandle n;
  if(argc > 1)
  {
  	ros::Subscriber sub = n.subscribe(argv[1], 1000, chatterCallback);
	ros::spin();
  }
  else
  {
	std::cout << "You must provide the topic to listen" << std::endl;
  }

  return 0;
}

