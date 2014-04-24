///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

// C++ standard
#include <algorithm>
#include <sstream>
#include <stdexcept>

// Orocos RTT
#include <rtt/Component.hpp>

// ros
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

// pal_ros_control
#include <pal_ros_control/ros_control_robot.h>

// Project
#include <ant_hardware/ant_hardware.h>

using std::vector;
using std::string;

namespace
{
const string ACT_POS_CUR_PORT        = "act_position";
const string ACT_VEL_CUR_PORT        = ""; //"act_velocity";
const string ACT_POS_REF_PORT        = ""; //"ref_position";
const string ACT_VEL_REF_PORT        = "ref_velocity";

#ifdef HAST_EMERGENCY_STOP
const string EMERGENCY_STOP_PORT     = "emergency_stop_state";
#endif
}

namespace ant_hardware
{

AntHardware::AntHardware(const string &name)
  : RTT::TaskContext(name, PreOperational)
  , actuators_(ACT_POS_CUR_PORT,
               ACT_VEL_CUR_PORT,
               ACT_POS_REF_PORT,
               ACT_VEL_REF_PORT,
               "", // No max current command interface
               this)
  , dummy_caster_data_(0.0)
#ifdef HAS_EMERGENCY_STOP
  , e_stop_(EMERGENCY_STOP_PORT,
            this)
#endif
  , do_stop_(false)
{}

bool AntHardware::configureHook()
{
  // Precondition
  if (isConfigured())
  {
    ROS_INFO_STREAM("Component is already configured. Cleaning up first.");
    cleanupHook();
  }

  // Setup actuators
  if (!actuators_.configure()) {return false;}

  // Setup callback queue and spinner thread
  ros::NodeHandle nh;
  nh.setCallbackQueue(&cb_queue_);
  spinner_.reset(new ros::AsyncSpinner(1, &cb_queue_));

  // Reset robot hardware abstraction
  try
  {
    robot_hw_.reset(new pal_ros_control::RosControlRobot(actuators_.getData(),
                                                         pal_ros_control::RawForceTorqueDataList(), // empty
                                                         pal_ros_control::RawImuDataList(), // empty
                                                         nh));
  }
  catch(const std::runtime_error& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unexpected exception caught while constructing robot hardware abstraction.");
    return false;
  }

  // Add dummy caster joints to robot hardware abstraction
  if (!addDummyCasters()) {return false;}

  // Reset controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_.get(), nh));

  // Start/stop services
  start_service_ = nh.advertiseService(getName() + "/start", &AntHardware::startService, this);
  stop_service_  = nh.advertiseService(getName() + "/stop",  &AntHardware::stopService,  this);

  // Start servicing ros callbacks
  spinner_->start();

  return true;
}

bool AntHardware::startHook()
{
  // Precondition: Configured component
  if (!isConfigured())
  {
    ROS_ERROR("Can't start component. It has not yet been configured.");
    return false;
  }

  // Precondition: Connected ports
  if (!actuators_.start())
#ifdef HAS_EMERGENCY_STOP
      //!e_stop_.start())// TODO: Add estop check?
#endif
  {
    return false;
  }

  // Controller restart variables
  restart_controllers_ = true;
#ifdef HAS_EMERGENCY_STOP
  e_stop_prev_val_     = false;
#endif

  // Check e-stop
#ifdef HAS_EMERGENCY_STOP
  if (!e_stop_.read())
  {
    // TODO: Enable!
//     ROS_ERROR("Can't start component: Couldn't read emergency stop state");
//     return false;
  }
#endif

  // Hold current position
  // TODO: Can we get rid of the error message at start of one lost cycle?
  if (!actuators_.read(ros::Duration(0.005))) // TODO: Remove magic number!
  {
    ROS_ERROR("Can't start component: Couldn't read actuators state");
    return false;
  }
  robot_hw_->holdPosition();

  // Initialize previous control cycle timestamp
  last_ticks_ = RTT::os::TimeService::Instance()->getTicks();

  return true;
}

void AntHardware::updateHook()
{
  // Stop if requested
  if (do_stop_)
  {
    do_stop_ = false;
    stop();
    return;
  }

  // Time management // TODO: Get time from actuators manager?
  using namespace RTT::os;
  TimeService::ticks ticks = TimeService::Instance()->getTicks(); // Current time
  ros::Time realtime_time; realtime_time.fromNSec(TimeService::ticks2nsecs(ticks));
  ros::Time time = realtime_clock_.getSystemTime(realtime_time);

  ros::Duration period; // Duration since last update
  period.fromNSec(TimeService::ticks2nsecs(TimeService::Instance()->ticksSince(last_ticks_)));

  last_ticks_ = ticks; // Cache current time

  // Emergency stop management
#ifdef HAS_EMERGENCY_STOP
  e_stop_.read();
  const bool estop_released = e_stop_prev_val_ && !e_stop_.getValue();
  if (!restart_controllers_ && estop_released)
#else
  if (!restart_controllers_)
#endif
  {
    restart_controllers_ = true;
    actuators_.resetVelocityEstimator();
  }
#ifdef HAS_EMERGENCY_STOP
  e_stop_prev_val_ = e_stop_.getValue(); // Cache current value for next update cycle

  if (e_stop_.getValue()) {return;} // Do nothing while in e-stop. TODO: Different policy?
#endif

  // Read current robot state
  const bool read_act_ok = actuators_.read(period);
  if (!read_act_ok) {return;}
  // TODO: Check sensors as well when bailing out?

  robot_hw_->read(time, period);

  // Compute the controller commands
  controller_manager_->update(time, period, restart_controllers_);
  restart_controllers_ = false;

  // Write commands
  robot_hw_->write(time, period);
  actuators_.write();

  // Update cycle time sanity check
  // TODO: Implement statistics publisher and don't log!
  if (getPeriod() > 0.0)
  {
    const double update_duration = TimeService::Instance()->secondsSince(ticks);
    if (update_duration > getPeriod())
    {
      ROS_ERROR_STREAM("Update cycle took " << update_duration <<
                       "s, which is greater than the control period of " << getPeriod() << "s.");
    }
  }
}

void AntHardware::stopHook()
{
  // Reset raw data and forward null commands to actuator manager
  actuators_.stop();
  actuators_.write(); // NOTE: Not really needed but a redundant safety measure
}

void AntHardware::cleanupHook()
{
  // Stop servicing ros callbacks
  spinner_->stop();

  // NOTE: Without this statement, unloading the component gives a:
  // "pure virtual method called, terminate called without an active exception. Aborted". Nasty.
  controller_manager_.reset();

  // Shutdown start/stop services
  start_service_.shutdown();
  stop_service_.shutdown();
}

bool AntHardware::addDummyCasters()
{
  // Preconditions
  if (!robot_hw_)
  {
    ROS_ERROR("Robot hardware abstraction not yet initialized. Not adding dummy casters.");
    return false;
  }
  hardware_interface::JointStateInterface* js_iface = robot_hw_->get<hardware_interface::JointStateInterface>();
  if (!js_iface)
  {
    ROS_ERROR("Robot hardware abstraction does not have a JointStates interface!. Not adding dummy casters.");
    return false;
  }

  // Add dummy casters
  double* dummy = &dummy_caster_data_;
  hardware_interface::JointStateHandle caster_left_1("caster_left_1_joint", dummy, dummy, dummy);
  hardware_interface::JointStateHandle caster_left_2("caster_left_2_joint", dummy, dummy, dummy);
  hardware_interface::JointStateHandle caster_right_1("caster_right_1_joint", dummy, dummy, dummy);
  hardware_interface::JointStateHandle caster_right_2("caster_right_2_joint", dummy, dummy, dummy);

  js_iface->registerHandle(caster_left_1);
  js_iface->registerHandle(caster_left_2);
  js_iface->registerHandle(caster_right_1);
  js_iface->registerHandle(caster_right_2);

  return true;
}

ORO_CREATE_COMPONENT(ant_hardware::AntHardware);

} // namespace
