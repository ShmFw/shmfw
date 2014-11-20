/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <gazebo/physics/physics.hh>
#include "gazebo_shm_laser.h"
#include <shmfw/allocator.h>
#include <shmfw/objects/ros/laser_scan.h>


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboShmLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboShmLaser::GazeboShmLaser()
: RayPlugin()
{
    shmHdl = ShmFw::Handler::create ( ShmFw::DEFAULT_SEGMENT_NAME(), ShmFw::DEFAULT_SEGMENT_SIZE() );
    shmScan.reset(new ShmFw::Alloc<ShmFw::ros::LaserScan>( "scan", shmHdl ));
    tstamp = boost::posix_time::microsec_clock::local_time();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboShmLaser::~GazeboShmLaser()
{
  this->parentSensor.reset();
}
////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboShmLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  this->parentSensor = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
}

////////////////////////////////////////////////////////////////////////////////
// Convert Gazebo message
void GazeboShmLaser::OnNewLaserScans()
{
  boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();       
  boost::posix_time::time_duration d = t - tstamp;
  tstamp = t;  
  ShmFw::ros::LaserScan &scan = *shmScan->get();
  std::vector<double> ranges;
  parentSensor->GetRanges(ranges);
  scan.ranges.resize(ranges.size());
  std::copy(ranges.begin(),  ranges.end(), scan.ranges.begin());
  scan.angle_min = parentSensor->GetAngleMin().Radian();
  scan.angle_max = parentSensor->GetAngleMax().Radian();
  scan.range_min = parentSensor->GetRangeMin();
  scan.range_max = parentSensor->GetRangeMax();
  scan.angle_increment = parentSensor->GetAngleResolution();
  
  scan.time_increment = 0;
  scan.scan_time = d.total_milliseconds();
  
  shmScan->itHasChanged();
  
}
