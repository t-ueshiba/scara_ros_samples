/*
 *  \file	scara_robot_driver.cpp
 *  \brief	ROS driver for Scara_Robot pan-tilt stage
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/controller_info.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <sys/types.h>
#include <errno.h>
#include "scara_lib.h"

namespace scara_robot_samples
{
/************************************************************************
*  class scara_robot_driver						*
************************************************************************/
class scara_robot_driver : public hardware_interface::RobotHW
{
  private:
    using joint_state_interface_t = hardware_interface::JointStateInterface;
    using position_joint_interface_t
		= hardware_interface::PositionJointInterface;
    using position_joint_saturation_interface_t
		= joint_limits_interface::PositionJointSaturationInterface;
    using manager_t = controller_manager::ControllerManager;
 
  public:
		scara_robot_driver()					;
    virtual	~scara_robot_driver()					;

    void	run()							;

    bool	init(ros::NodeHandle& root_nh, ros::NodeHandle& nh)	;
    void	read(const ros::Time&, const ros::Duration&)		;
    void	write(const ros::Time&, const ros::Duration& period)	;

  private:
    ros::NodeHandle			  _root_nh;
    ros::NodeHandle			  _nh;

    HID_UART_DEVICE			  _dev;

    joint_state_interface_t		  _joint_state_interface;
    position_joint_interface_t		  _position_joint_interface;
    position_joint_saturation_interface_t _position_joint_saturation_interface;

    std::array<double, 5>		  _position;
    std::array<double, 5>		  _velocity;
    std::array<double, 5>		  _effort;
    std::array<double, 5>		  _position_command;

    static constexpr int		  TRANSITION_MSEC = 150;
};

scara_robot_driver::scara_robot_driver()
    :_root_nh(),
     _nh("~"),
     _dev(0),
     _position{0.0, 0.0, 0.0, 0.0, 0.0},
     _velocity{0.0, 0.0, 0.0, 0.0, 0.0},
     _effort{0.0, 0.0, 0.0, 0.0, 0.0},
     _position_command{0.0, 0.0, 0.0, 0.0, 0.0}
{
  // Register joint handles and interfaces.
    init(_root_nh, _nh);

  // Set debug level and control mode and then do calibaration.
    unsigned	ndevices = 0;
    HidUart_GetNumDevices(&ndevices, VID, PID);
    if (ndevices < 1)
	throw std::runtime_error("(scala_robot_driver) no devices found!");
    if (HidUart_Open(&_dev, 0, VID, PID) != HID_UART_SUCCESS)
	throw std::runtime_error(
	    "(scala_robot_driver) failed to open device!");
    if (SetTXOpenDrain(_dev) != HID_UART_SUCCESS)
	throw std::runtime_error(
	    "(scala_robot_driver) failed to set TX to open-drain!");
    if (!RSTorqueOnOff(_dev, 1, 1, 5))
	throw std::runtime_error(
	    "(scala_robot_driver) failed to tuun on torque!");

    ROS_INFO_STREAM("(scara_robot_driver) scara_robot_driver initialized.");
}

scara_robot_driver::~scara_robot_driver()
{
    if (_dev != 0)
    {
	RSTorqueOnOff(_dev, 0, 1, 5);
	HidUart_Close(_dev);
    }
}

void
scara_robot_driver::run()
{
    manager_t		manager(this, _root_nh);
    ros::Rate		rate(_nh.param<double>("rate", 100.0));
    ros::AsyncSpinner	spinner(1);
    spinner.start();

    ROS_INFO_STREAM("(scara_robot_driver) scara_robot_driver started.");

    for (auto timestamp = ros::Time::now(); ros::ok(); )
    {
	read(timestamp, rate.cycleTime());
	timestamp = ros::Time::now();
	manager.update(timestamp, rate.cycleTime());
	write(timestamp, rate.cycleTime());
	rate.sleep();
    }

    spinner.stop();
}

bool
scara_robot_driver::init(ros::NodeHandle& root_nh, ros::NodeHandle& nh)
{
    using joint_state_handle_t	= hardware_interface::JointStateHandle;
    using joint_handle_t	= hardware_interface::JointHandle;
    using joint_limits_t	= joint_limits_interface::JointLimits;

    const auto	prefix = nh.param<std::string>("prefix", "");
    const char*	joint_names[] = {"arm1_joint", "arm2_joint",
				 "arm3_joint", "arm4_joint", "arm5r_joint"};

    for (size_t i = 0; i < _position.size(); ++i)
    {
	joint_state_handle_t	joint_state_handle(prefix + joint_names[i],
						   &_position[i],
						   &_velocity[i],
						   &_effort[i]);
	_joint_state_interface.registerHandle(joint_state_handle);

	joint_limits_t	limits;
	joint_limits_interface::getJointLimits(prefix + joint_names[i],
					       root_nh, limits);

	joint_handle_t	joint_handle(joint_state_handle,
				     &_position_command[i]);
	_position_joint_interface.registerHandle(joint_handle);
	_position_joint_saturation_interface.registerHandle({joint_handle,
							     limits});
    }

    registerInterface(&_joint_state_interface);
    registerInterface(&_position_joint_interface);
    registerInterface(&_position_joint_saturation_interface);
}

void
scara_robot_driver::read(const ros::Time&, const ros::Duration&)
{
    for (size_t i = 0; i < _position.size(); ++i)
    {
	short	pos;
	if (RSGetAngle(_dev, i + 1, &pos))
	{
	    switch (i)
	    {
	      case 2:
		_position[i] = RAD_TO_HEIGHT(pos) / 1000.0;
		break;
	      case 4:
		_position[i] = -(RAD_TO_WIDTH(pos, CROW_POS) - 20)/2000;
		break;
	      default:
		_position[i] = double(pos) * M_PI/1800.0;
		break;
	    }
	}
	else
	{
	    ROS_ERROR_STREAM(
		"(scara_robot_driver) failed to get status of axis["
		<< i + 1 << ']');
	}
    }
}

void
scara_robot_driver::write(const ros::Time&, const ros::Duration& period)
{
    _position_joint_saturation_interface.enforceLimits(period);

    short	pos[5];
    for (size_t i = 0; i < _position_command.size(); ++i)
	switch (i)
	{
	  case 2:
	    pos[i] = HEIGHT_TO_RAD(_position_command[i] * 1000.0);
	    break;
	  case 4:
	    pos[i] = WIDTH_TO_RAD((-_position_command[i] * 2000.0) + 20,
				  CROW_POS);
	    break;
	  default:
	    pos[i] = short(_position_command[i] * 1800.0/M_PI);
	    break;
	}

    if (!RSMove(_dev, pos, TRANSITION_MSEC, 1, 5))
	ROS_ERROR_STREAM("(scara_robot_driver) failed to move axes");
}

}	// namepsace scara_robot_samples

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "scara_robot_driver");

    try
    {
	scara_robot_samples::scara_robot_driver	node;
	node.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }
    catch (...)
    {
	std::cerr << "Unknown error." << std::endl;
	return 1;
    }

    return 0;
}
