
#include <arduino_daq/ArduinoDAQ_LowLevel.h>
#include <std_msgs/String.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "arduino_daq");

		ArduinoDAQ_LowLevel  daq;
		daq.initialize();

		ros::Rate loop_rate(100);
		while (ros::ok())
		{
			ros::spinOnce();
			daq.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
