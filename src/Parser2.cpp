/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Walter Nowak
 * Sushant Chavan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include <iostream>
#include <yaml_common/Parser2.h>

namespace kelo::yaml_common
{

template <typename T>
bool Parser2::read(const YAML::Node& node, const std::string& key,
                   T& value, bool print_error_msg)
{
    if ( !Parser2::performSanityChecks(node, key, print_error_msg) )
    {
        return false;
    }
    if ( !read(node[key], value, print_error_msg) )
    {
        if ( print_error_msg )
        {
            std::cout << "[Parser2] Could not read YAML::Node with key "
                      << key << std::endl;
        }
        return false;
    }
    return true;
}
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            int& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            float& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            double& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            unsigned int& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            bool& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            std::string& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            geometry_common::Point2D& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            geometry_common::Point3D& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            geometry_common::XYTheta& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            geometry_common::Pose2D& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            geometry_common::TransformMatrix2D& value,
                            bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, const std::string& key,
                            geometry_common::TransformMatrix3D& value,
                            bool print_error_msg);

template <typename T>
bool Parser2::read(const YAML::Node& node, T& value, bool print_error_msg)
{
    if ( !node.IsScalar() )
    {
        if ( print_error_msg )
        {
            std::cout << "[Parser2] Given YAML::Node is not scalar" << std::endl;
        }
        return false;
    }
    
    try
    {
        value = node.as<T>();
    }
    catch ( YAML::Exception& )
    {
        if ( print_error_msg )
        {
            std::cout << "[Parser2] Could not read value of YAML::Node" << std::endl;
        }
        return false;
    }

    return true;
}
template bool Parser2::read(const YAML::Node& node, int& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, float& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, double& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, unsigned int& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, bool& value, bool print_error_msg);
template bool Parser2::read(const YAML::Node& node, std::string& value, bool print_error_msg);

bool Parser2::read(const YAML::Node& node, geometry_common::Point2D& value,
                   bool print_error_msg)
{
    std::vector<std::string> keys{"x", "y"};
    std::vector<float> values;
    if ( !Parser2::readFloats(node, keys, values, print_error_msg) )
    {
        return false;
    }

    value.x = values[0];
    value.y = values[1];
    return true;
}

bool Parser2::read(const YAML::Node& node, geometry_common::Point3D& value,
                   bool print_error_msg)
{
    std::vector<std::string> keys{"x", "y", "z"};
    std::vector<float> values;
    if ( !Parser2::readFloats(node, keys, values, print_error_msg) )
    {
        return false;
    }

    value.x = values[0];
    value.y = values[1];
    value.z = values[2];
    return true;
}

bool Parser2::read(const YAML::Node& node, geometry_common::XYTheta& value,
                   bool print_error_msg)
{
    std::vector<std::string> keys{"x", "y", "theta"};
    std::vector<float> values;
    if ( !Parser2::readFloats(node, keys, values, print_error_msg) )
    {
        return false;
    }

    value.x = values[0];
    value.y = values[1];
    value.theta = values[2];
    return true;
}

bool Parser2::read(const YAML::Node& node, geometry_common::Pose2D& value,
                   bool print_error_msg)
{
    geometry_common::XYTheta temp_value;
    if ( !Parser2::read(node, temp_value, print_error_msg) )
    {
        return false;
    }
    value = geometry_common::Pose2D(temp_value); // does angle clipping
    return true;
}

bool Parser2::read(const YAML::Node& node, geometry_common::TransformMatrix2D& value,
                   bool print_error_msg)
{
    std::vector<std::string> keys{"x", "y", "theta"};
    std::vector<std::string> keys_quat{"x", "y", "qx", "qy", "qz", "qw"};
    std::vector<float> values;

    if ( Parser2::readFloats(node, keys, values, false) )
    {
        value.update(values[0], values[1], values[2]);
        return true;
    }

    values.clear();
    if ( Parser2::readFloats(node, keys_quat, values, false) )
    {
        value.update(values[0], values[1], values[2],
                     values[3], values[4], values[5]);
        return true;
    }

    if ( print_error_msg )
    {
        std::cout << "[Parser2] Could not read TransformMatrix2D with"
                  << " either euler or quaternion format." << std::endl;
    }
    return false;
}

bool Parser2::read(const YAML::Node& node, geometry_common::TransformMatrix3D& value,
                   bool print_error_msg)
{
    std::vector<std::string> keys{"x", "y", "z", "roll", "pitch", "yaw"};
    std::vector<std::string> keys_quat{"x", "y", "z", "qx", "qy", "qz", "qw"};
    std::vector<float> values;
    if ( Parser2::readFloats(node, keys, values, false) )
    {
        value.update(values[0], values[1], values[2],
                     values[3], values[4], values[5]);
        return true;
    }

    values.clear();
    if ( Parser2::readFloats(node, keys_quat, values, false) )
    {
        value.update(values[0], values[1], values[2],
                     values[3], values[4], values[5], values[6]);
        return true;
    }

    if ( print_error_msg )
    {
        std::cout << "[Parser2] Could not read TransformMatrix3D with"
                  << " either euler or quaternion format." << std::endl;
    }
    return false;
}

bool Parser2::readFloats(
        const YAML::Node& node, const std::vector<std::string>& keys,
        std::vector<float>& values, bool print_error_msg)
{
    values.resize(keys.size());
    for ( size_t i = 0; i < keys.size(); i++ )
    {
        if ( !Parser2::read<float>(node, keys[i], values[i], print_error_msg) )
        {
            return false;
        }
    }
    return true;
}

bool Parser2::performSanityChecks(const YAML::Node& node, const std::string& key,
                                  bool print_error_msg)
{
    if ( key.empty() )
    {
        if ( print_error_msg )
        {
            std::cout << "[Parser2] Given key is empty" << std::endl;
        }
        return false;
    }

    if ( !node.IsMap() )
    {
        if ( print_error_msg )
        {
            std::cout << "[Parser2] Given YAML::Node is not a map" << std::endl;
        }
        return false;
    }

    if ( !node[key] )
    {
        if ( print_error_msg )
        {
            std::cout << "[Parser2] Given YAML::Node does not have key "
                      << key << std::endl;
        }
        return false;
    }

    return true;
}

} // namespace kelo::yaml_common
