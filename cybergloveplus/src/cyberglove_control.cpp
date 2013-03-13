#include "cybergloveplus/cyberglove_control.h"

namespace CyberGlovePlus
{

CyberGloveControl::CyberGloveControl(bool biotac)
{
	_is_biotac = biotac;
}

CyberGloveControl::~CyberGloveControl()
{
}

int CyberGloveControl::init()
{
	int result = CyberGloveCalibMinMax::init();	
	if (result != 0)
		return result;

	 static const std::string joint_names[20] = {"FFJ0", "FFJ3", "FFJ4",
                                           "MFJ0", "MFJ3", "MFJ4",
                                           "RFJ0", "RFJ3", "RFJ4",
                                           "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                                           "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                                           "WRJ1", "WRJ2"};

	static const std::string prefix = "/sh_";
	std::string suffix = "_position_controller/command";

	if (n_tilde.hasParam("controller_suffix"))
	{
		n_tilde.getParam("controller_suffix",suffix);
	}
	else if (n_tilde.hasParam("/controller_suffix"))
	{
		n_tilde.getParam("/controller_suffix", suffix);
	}
	ROS_INFO("Controller suffix is %s", suffix.c_str());



	for (int i = 0; i < JOINTS_SIZE; i++)
	{
		std::string joint = joint_names[i];
		joints.push_back(joint);
		
		std::string joint_temp = tolower(joint);
        publisher[joint] = n_tilde.advertise<std_msgs::Float64>( prefix + joint_temp + suffix , 2 );
	}

	init_limit_angles();


	for (int i = 0; i < JOINTS_SIZE; i++)
	{
		bool active = DEFAULT_JOINT_ACTIVE;
		std::string joint = joints[i];
		std::string joint_param = tolower(joint) + "_active";
		std::string prefix = "/";
		std::string joint_param_global = prefix + joint_param;
		if (n_tilde.hasParam(joint_param.c_str()))
		{
			n_tilde.getParam(joint_param.c_str(), active);
		}
		else if (n_tilde.hasParam(joint_param_global.c_str()))
		{
			n_tilde.getParam(joint_param_global.c_str(), active);
		}
		joints_active[joint] = active;
		if (active)
			ROS_INFO("Joint %s activated", joint.c_str());
		else
			ROS_INFO("Joint %s disactivated", joint.c_str());
	}



	return 0;
}

void CyberGloveControl::run()
{
	CyberGloveCalibMinMax::run();
}

void CyberGloveControl::init_limit_angles()
{
		if (_is_biotac)
		{
        	min_angles["FFJ0"] = 20;    max_angles["FFJ0"] = 110;
        }
        else
        {
        	min_angles["FFJ0"] = 0;     max_angles["FFJ0"] = 180;
        }
        min_angles["FFJ3"] = 0;         max_angles["FFJ3"] = 90;
        min_angles["FFJ4"] = -25;       max_angles["FFJ4"] = 25;

		
		if (_is_biotac)
		{
	        min_angles["MFJ0"] = 20;    max_angles["MFJ0"] = 110;
	    }
        else
        {
   	        min_angles["MFJ0"] = 0;     max_angles["MFJ0"] = 180;
   	    }
     	min_angles["MFJ3"] = 0;         max_angles["MFJ3"] = 90;
        min_angles["MFJ4"] = -25;       max_angles["MFJ4"] = 25;

		if (_is_biotac)
		{
	        min_angles["RFJ0"] = 20;    max_angles["RFJ0"] = 110;
	    }
        else
        {
   	        min_angles["RFJ0"] = 0;     max_angles["RFJ0"] = 180;
   	    }
   	  	min_angles["RFJ3"] = 0;         max_angles["RFJ3"] = 90;
        min_angles["RFJ4"] = -25;       max_angles["RFJ4"] = 25;

		if (_is_biotac)
		{
	        min_angles["LFJ0"] = 20;    max_angles["LFJ0"] = 110;
	    }
        else
        {
       		min_angles["LFJ0"] = 0;     max_angles["LFJ0"] = 180;
        }
        min_angles["LFJ3"] = 0;         max_angles["LFJ3"] = 90;
        min_angles["LFJ4"] = -25;       max_angles["LFJ4"] = 25;
        min_angles["LFJ5"] = 0;         max_angles["LFJ5"] = 45;

        min_angles["THJ1"] = 0;         max_angles["THJ1"] = 90;
        min_angles["THJ2"] = -30;       max_angles["THJ2"] = 30;
        min_angles["THJ3"] = -15;       max_angles["THJ3"] = 15;
        min_angles["THJ4"] = 0;         max_angles["THJ4"] = 70;
        min_angles["THJ5"] = -60;       max_angles["THJ5"] = 60;

        min_angles["WRJ1"] = -45;		max_angles["WRJ1"] = 35;
        min_angles["WRJ2"] = -30;		max_angles["WRJ2"] = 10;
        }


void CyberGloveControl::action()
{
	CyberGloveCalibMinMax::action();

	update_commands();

	send_commands();	
}

void CyberGloveControl::update_commands()
{
	if (_is_biotac)
	{
		linear_one_one("FFJ0", "G_IndexPIJ");
		linear_one_one("MFJ0", "G_MiddlePIJ");
		linear_one_one("RFJ0", "G_RingPIJ");
		linear_one_one("LFJ0", "G_PinkiePIJ");
	}
	else
	{
		linear_one_two("FFJ0", "G_IndexPIJ", "G_IndexDIJ");
		linear_one_two("MFJ0", "G_MiddlePIJ", "G_MiddleDIJ");
		linear_one_two("RFJ0", "G_RingPIJ", "G_RingDIJ");
		linear_one_two("LFJ0", "G_PinkiePIJ", "G_PinkieDIJ");
	}


    linear_one_one("FFJ3", "G_IndexMPJ");
    linear_one_one("MFJ3", "G_MiddleMPJ");
    linear_one_one("RFJ3", "G_RingMPJ");
    linear_one_one("LFJ3", "G_PinkieMPJ");

	linear_one_one("FFJ4", "G_MiddleIndexAb");
	linear_one_one("RFJ4", "G_RingMiddleAb");
	linear_one_one("LFJ4", "G_PinkieRingAb");

	if (_is_biotac)
	{
		linear_one_one("THJ2", "G_ThumbIJ");
		linear_one_one("THJ4", "G_ThumbAb");
	}
	else
	{
		linear_one_one("THJ1", "G_ThumbIJ");
		linear_one_one("THJ2", "G_ThumbMPJ");
		linear_one_one("THJ4", "G_ThumbAb");
	}

	// in the glove, the abduction is 0 when the thumb is at maximum abduction. in the shadow hand the maximum abduction is the opposite value (maximum)
	j_values["THJ4"] = 1 - j_values["THJ4"];

	linear_one_one("THJ5", "G_ThumbRotate");

	linear_one_one("WRJ1", "G_WristPitch");
	linear_one_one("WRJ2", "G_WristYaw");
	
	// wrj2 is also inversed like thj4
	j_values["WRJ2"] = 1 - j_values["WRJ2"];

	// there is no sensor for thj3. so it is set to straight
	j_values["THJ3"] = 0.5;

	j_previous_commands = j_commands;
	for (int i = 0; i < JOINTS_SIZE; i++)
	{
		std::string joint = joints[i];
		j_commands[joint] = get_radiant_value(joint, j_values[joint]);
	}

	// mfj4 is set to 0 always so that we can map 3 glove sensors (ab) to 3 joints 4 (+1 that is mfj4)
	j_commands["MFJ4"] = 0;

	
	// to make sure that the fingers do not cross
	if (j_commands["FFJ4"] > j_commands["MFJ4"])
		j_commands["FFJ4"] = j_commands["MFJ4"];

	if (j_commands["RFJ4"] > j_commands["MFJ4"])
		j_commands["RFJ4"] = j_commands["MFJ4"];

	if (j_commands["LFJ4"] > j_commands["RFJ4"])
		j_commands["LFJ4"] = j_commands["RFJ4"];
}

void CyberGloveControl::send_commands()
{
	for (int i = 0; i < JOINTS_SIZE; i++)
	{
		std::string joint = joints[i];
		if (gl_button == 1 && joints_active[joint])
			send_command(joint, j_commands[joint]);
	}
}

bool CyberGloveControl::send_command(std::string joint_name, double target)
{
	std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	publisher_iterator = publisher.find(joint_name);

	if( publisher_iterator == publisher.end() )
	{
		ROS_WARN_STREAM("Joint "<< joint_name << " not found.");
		return false;
	}

	double max = max_angles[joint_name] * rad_ang;
	double min = min_angles[joint_name] * rad_ang;

	if (min > target)
	{
		ROS_WARN_STREAM("Command is smaller than minimum allowed for joint " << joint_name << " min is " << min << " received " << target);
		target = min;
	}
	
	if (max < target)
	{
		ROS_WARN_STREAM("Command is larger than maximum allowed for joint " << joint_name << " max is " << max << " received " << target);
		target = max;
	}

	std_msgs::Float64 msg_to_send;
	msg_to_send.data = target;
	publisher_iterator->second.publish( msg_to_send );

    return true;
}

void CyberGloveControl::linear_one_one(std::string joint, std::string sensor)
{
	j_values[joint] = gl_calibrated_values[sensor];
}

void CyberGloveControl::linear_one_two(std::string joint, std::string sensor1, std::string sensor2)
{
	j_values[joint] = (gl_calibrated_values[sensor1] + gl_calibrated_values[sensor2]) / 2;
}

double CyberGloveControl::get_radiant_value(std::string joint, double value)
{
	double max = max_angles[joint] * rad_ang;
	double min = min_angles[joint] * rad_ang;

	return value * (max - min) + min;
}


}
