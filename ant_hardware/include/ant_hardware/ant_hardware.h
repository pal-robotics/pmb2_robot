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

#ifndef ANT_HARDWARE_ANT_HARDWARE_H
#define ANT_HARDWARE_ANT_HARDWARE_H

// C++ standard
#include <vector>
#include <string>

// Boost
#include <boost/scoped_ptr.hpp>

// Orocos RTT
#include <rtt/TaskContext.hpp>
#include <rtt/os/TimeService.hpp>

// ros
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>

// pal_ros_control
#include <pal_ros_control/hardware_accessors.h>

// realtime_tools
#include <realtime_tools/realtime_clock.h>

namespace controller_manager
{
  class ControllerManager;
}

namespace pal_ros_control
{
  class RosControlRobot;
}

namespace ant_hardware
{

class AntHardware : public RTT::TaskContext
{
public:
  AntHardware(const std::string& name);

protected:
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();

private:
  // Actuators manager interface
  pal_ros_control::ActuatorAccesor actuators_;
  double dummy_caster_data_; // Dummy raw caster data

  // Emergency stop
#ifdef HAS_EMERGENCY_STOP
  pal_ros_control::EmergencyStopAccesor e_stop_;
  bool e_stop_prev_val_;
#endif
  bool restart_controllers_;

  // Time management
  RTT::os::TimeService::ticks last_ticks_;
  realtime_tools::RealtimeClock realtime_clock_;

  // ros_control - robot hardware
  boost::scoped_ptr<pal_ros_control::RosControlRobot> robot_hw_;

  // ros_control - controller manager
  boost::scoped_ptr<controller_manager::ControllerManager> controller_manager_;

  // ROS callback processing
  ros::CallbackQueue cb_queue_;
  boost::scoped_ptr<ros::AsyncSpinner> spinner_;

  // ROS services for starting/stopping
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  bool do_stop_; /// Flag used to signal a pending stop

  bool startService(std_srvs::Empty::Request&  req,
                    std_srvs::Empty::Response& resp)
  {
    if (!isRunning()) {start();}
    return true;
  }

  bool stopService(std_srvs::Empty::Request&  req,
                   std_srvs::Empty::Response& resp)
  {
    if (isRunning()) {do_stop_ = true;}
    return true;
  }

  /**
   * \brief Add dummy caster joints to robot hardware abstraction.
   * Caster joints don't have sensors, so we set them to zero and make them show up in joint_states mostly for cosmetic
   * reasons (otherwise things like tf and the RobotModel Rviz plugin can't update the frame locations).
   */
  bool addDummyCasters();
};

} // namespace

#endif // header guard
