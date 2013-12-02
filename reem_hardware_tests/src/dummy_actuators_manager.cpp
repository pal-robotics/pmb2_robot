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
#include <cmath>
#include <limits>
#include <vector>
#include <string>

// Orocos RTT
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Property.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>


namespace reem_hardware
{
using std::string;
using std::vector;

namespace
{
const string POS_CUR_PORT            = "act_position";
const string VEL_CUR_PORT            = "act_velocity";
const string POS_REF_PORT            = "ref_position";
const string VEL_REF_PORT            = "ref_velocity";
const string BASE_ORIENTATION_PORT   = "orientation_base";
const string GET_ACTUATORS_NAMES     = "getActuatorsNames";

const string ACT_MANAGER_NAME   = "actuatorsMgr";
}

class DummyActuatorsManager : public RTT::TaskContext
{
public:
  DummyActuatorsManager(const string& name)
    : RTT::TaskContext(ACT_MANAGER_NAME)
  {
    // Raw data
    act_names_.push_back("wheel_right_motor");
    act_names_.push_back("wheel_left_motor");

    act_names_.push_back("caster_left_1_motor");
    act_names_.push_back("caster_left_2_motor");
    act_names_.push_back("caster_right_1_motor");
    act_names_.push_back("caster_right_2_motor");

    act_names_.push_back("torso_1_motor");
    act_names_.push_back("torso_2_motor");
    act_names_.push_back("head_1_motor");
    act_names_.push_back("head_2_motor");

    act_names_.push_back("arm_left_1_motor");
    act_names_.push_back("arm_left_2_motor");
    act_names_.push_back("arm_left_3_motor");
    act_names_.push_back("arm_left_4_motor");
    act_names_.push_back("arm_left_5_motor");
    act_names_.push_back("arm_left_6_motor");
    act_names_.push_back("arm_left_7_motor");
    act_names_.push_back("arm_right_1_motor");
    act_names_.push_back("arm_right_2_motor");
    act_names_.push_back("arm_right_3_motor");
    act_names_.push_back("arm_right_4_motor");
    act_names_.push_back("arm_right_5_motor");
    act_names_.push_back("arm_right_6_motor");
    act_names_.push_back("arm_right_7_motor");

    act_names_.push_back("hand_left_thumb_motor");
    act_names_.push_back("hand_left_index_motor");
    act_names_.push_back("hand_left_index_1_motor");
    act_names_.push_back("hand_left_index_2_motor");
    act_names_.push_back("hand_left_index_3_motor");
    act_names_.push_back("hand_left_middle_motor");
    act_names_.push_back("hand_left_middle_1_motor");
    act_names_.push_back("hand_left_middle_2_motor");
    act_names_.push_back("hand_left_middle_3_motor");
    act_names_.push_back("hand_right_thumb_motor");
    act_names_.push_back("hand_right_index_motor");
    act_names_.push_back("hand_right_index_1_motor");
    act_names_.push_back("hand_right_index_2_motor");
    act_names_.push_back("hand_right_index_3_motor");
    act_names_.push_back("hand_right_middle_motor");
    act_names_.push_back("hand_right_middle_1_motor");
    act_names_.push_back("hand_right_middle_2_motor");
    act_names_.push_back("hand_right_middle_3_motor");
    
    act_names_.push_back("zzz_can_sync");

    const unsigned int dim = act_names_.size();
    const double nan_val = std::numeric_limits<double>::quiet_NaN();
    pos_.resize(dim, 0.0);
    vel_.resize(dim, 0.0);
    eff_.resize(dim, 0.0);
    pos_cmd_.resize(dim, nan_val);
    vel_cmd_.resize(dim, nan_val);
    orientation_.resize(4, 0.0); orientation_[3] = 1.0;

    // Data flow interface
    ports()->addPort(POS_CUR_PORT, pos_ref_port_);
    ports()->addPort(VEL_CUR_PORT, vel_ref_port_); // NOTE: Comment!
    ports()->addPort(POS_REF_PORT, pos_cmd_port_);
    ports()->addPort(VEL_REF_PORT, vel_cmd_port_);
    pos_ref_port_.setDataSample(pos_); // Preallocate resources
    vel_ref_port_.setDataSample(vel_); // Preallocate resources

    ports()->addPort(BASE_ORIENTATION_PORT, orientation_port_);
    orientation_port_.setDataSample(orientation_);  // Preallocate resources

    provides()->addOperation(GET_ACTUATORS_NAMES,
                             &DummyActuatorsManager::getActuatorsNames,
                             this,
                             RTT::ClientThread)
    .doc("returns the names of the managed actuators");

  }

  vector<string> getActuatorsNames() const {return act_names_;}

protected:
  void updateHook()
  {
    if (pos_cmd_port_.read(pos_cmd_) == RTT::NewData)
    {
      // Estimate velocity from command and current position (previous command)
      for (unsigned int i = 0; i < pos_.size(); ++i)
      {
        // Ignore non-controlled joints
        if (std::isnan(pos_cmd_[i])) {continue;}

        // Apply exponential smoothing to previous position estimate
        const double smoothing = 0.1;
        const double smooth_pos = (1.0 - smoothing) * pos_[i] + smoothing * pos_cmd_[i];

        vel_[i] = (pos_cmd_[i] - smooth_pos) / getPeriod();

        // Echo command to current state: perfect control
        pos_[i] = pos_cmd_[i];
      }
    }

    if (vel_cmd_port_.read(vel_cmd_) == RTT::NewData)
    {
      for (unsigned int i = 0; i < pos_.size(); ++i)
      {
        // Ignore non-controlled joints
        if (std::isnan(vel_cmd_[i])) {continue;}
        pos_[i] = pos_[i] + vel_cmd_[i] * getPeriod(); // Not robust to update misses!
        vel_[i] = vel_cmd_[i];
      }
    }

    // Write current state
    pos_ref_port_.write(pos_);
    vel_ref_port_.write(vel_); // NOTE: Comment!

    orientation_port_.write(orientation_);
  }

private:

  // Raw data
  vector<string> act_names_;
  vector<double> pos_;
  vector<double> vel_;
  vector<double> eff_;
  vector<double> pos_cmd_;
  vector<double> vel_cmd_;
  vector<double> orientation_;

  // Actuators manager interface
  RTT::OutputPort< vector<double> >  pos_ref_port_;
  RTT::OutputPort< vector<double> >  vel_ref_port_;
  RTT::InputPort< vector<double> >   pos_cmd_port_;
  RTT::InputPort< vector<double> >   vel_cmd_port_;
  RTT::OutputPort< vector<double> >  orientation_port_;
};

ORO_CREATE_COMPONENT(reem_hardware::DummyActuatorsManager);
}
