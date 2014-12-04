/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at>
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

#ifndef GAZEBO_SHM_UTILS_H
#define GAZEBO_SHM_UTILS_H
#include <map>
#include <boost/algorithm/string.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{

/**
* Accessing model name like suggested by nkoenig at http://answers.gazebosim.org/question/4878/multiple-robots-with-ros-plugins-sensor-plugin-vs/
* @param parent
* @return accessing model name
**/
inline std::string GetModelName ( const sensors::SensorPtr &parent )
{
    std::string modelName;
    std::vector<std::string> values;
    std::string scopedName = parent->GetScopedName();
    boost::replace_all ( scopedName, "::", "," );
    boost::split ( values, scopedName, boost::is_any_of ( "," ) );
    if ( values.size() < 2 ) {
        modelName = "";
    } else {
        modelName = values[1];
    }
    return modelName;
}

/**
* @brief Reads the name space tag of a sensor plugin
* @param parent
* @param sdf
* @param pInfo
* @return node namespace
**/
inline std::string GetRobotNamespace ( const sensors::SensorPtr &parent, const sdf::ElementPtr &sdf, const char *pInfo = NULL )
{
    std::string name_space;
    std::stringstream ss;
    if ( sdf->HasElement ( "robotNamespace" ) ) {
        name_space = sdf->Get<std::string> ( "robotNamespace" );
        if ( name_space.empty() ) {
            ss << "the 'robotNamespace' param was empty";
            name_space = GetModelName ( parent );
        } else {
            ss << "Using the 'robotNamespace' param: '" <<  name_space << "'";
        }
    } else {
        ss << "the 'robotNamespace' param did not exit";
    }
    if ( pInfo != NULL ) {
        ///@ToDo GAZ_INFO ( "%s Plugin (robotNamespace = %s), Info: %s" , pInfo, name_space.c_str(), ss.str().c_str() );
    }
    return name_space;
}

#endif



