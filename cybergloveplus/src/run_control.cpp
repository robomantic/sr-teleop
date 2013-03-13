#include <ros/ros.h>
#include "cybergloveplus/cyberglove_control.h"

int main(int argc, char **argv)
{
        ros::init(argc, argv, "cybergloveplus_control");

		ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		ROS_WARN("!!!  PLEASE FOLLOW THE FOLLOWING STEPS IN ORDER TO USE THE TELEOP        ");
		ROS_WARN("!!!  1. MAKE SURE THAT THE GLOVE LED IS OFF. IF IS ON PLEASE USE BUTTON  ");
		ROS_WARN("!!!  2. DO CALIBRATION MOVEMENTS FOR ALL FINGERS AND JOINTS (including wrist joints)");
		ROS_WARN("!!!  3. START THE TELEOP BY USING THE BUTTTON (THE LED WILL BECOME ON)");

		for (int i = 10; i > 0; i--)
		{
			ROS_WARN("Starting in %d seconds. Hurry up!!!!", i);
			if (i == 1)
				ROS_ERROR("Starting immediately!!!!");

			sleep(1);
		}

		bool is_biotac = true;
		ros::NodeHandle n_tilde("~");

		if (n_tilde.hasParam("is_biotac"))
		{
			n_tilde.getParam("is_biotac", is_biotac);
		}
		else if (n_tilde.hasParam("/is_biotac"))
		{
			n_tilde.getParam("/is_biotac", is_biotac);
		}

		CyberGlovePlus::CyberGloveControl glove(is_biotac);

        int res;
        if ((res = glove.init()) == 0)
        {
                glove.run();
        }
        else
        {
                ROS_ERROR("Could not init glove plus");
				return 1;
        }

    return 0;
}

