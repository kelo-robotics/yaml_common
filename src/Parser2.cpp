/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin B.
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

namespace kelo
{
namespace yaml_common
{

bool Parser2::loadFile(const std::string& abs_file_path,
                       YAML::Node& node, bool print_error_msg)
{
    try
    {
        node = YAML::LoadFile(abs_file_path);
    }
    catch( const YAML::BadFile& )
    {
        std::stringstream msg;
        msg << "YAML threw BadFile exception. Does the file exist?"
            << std::endl << abs_file_path;
        Parser2::log(msg.str(), print_error_msg);
        return false;
    }
    catch( const YAML::ParserException& e )
    {
        std::stringstream msg;
        msg << "YAML parsing error" << std::endl << e.what();
        Parser2::log(msg.str(), print_error_msg);
        return false;
    }
    return true;
}

bool Parser2::readAllKeys(const YAML::Node& node, std::vector<std::string>& keys,
                          bool print_error_msg)
{
    if ( !node.IsMap() )
    {
        Parser2::log("Given YAML::Node is not a map", print_error_msg);
        return false;
    }

    for ( const auto& kv : node )
    {
        if ( kv.first.IsScalar() )
        {
            keys.push_back(kv.first.Scalar());
        }
        else
        {
            Parser2::log("Given YAML::Node map contains a non-scalar key", print_error_msg);
            return false;
        }
    }
    return true;
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

bool Parser2::hasKey(const YAML::Node& node, const std::string& key,
                     bool print_error_msg)
{
    if ( key.empty() )
    {
        Parser2::log("Given key is empty", print_error_msg);
        return false;
    }

    if ( !node.IsMap() )
    {
        Parser2::log("Given YAML::Node is not a map", print_error_msg);
        return false;
    }

    if ( !node[key] )
    {
        std::stringstream msg;
        msg << "Given YAML::Node does not have key " << key;
        Parser2::log(msg.str(), print_error_msg);
        return false;
    }

    return true;
}

YAML::Node Parser2::mergeYAML(const YAML::Node& base_node,
                              const YAML::Node& override_node)
{
    /**
     * source: https://stackoverflow.com/a/41337824/10460994
     */
    if ( !override_node.IsMap() )
    {
        // If override_node is not a map, merge result is override_node, unless override_node is null
        return override_node.IsNull() ? base_node : override_node;
    }

    if ( !base_node.IsMap() )
    {
        // If base_node is not a map, merge result is override_node
        return override_node;
    }

    if ( !base_node.size() )
    {
        return YAML::Node(override_node);
    }

    /* Create a new map 'new_node' with the same mappings as base_node,
     * merged with override_node */
    auto new_node = YAML::Node(YAML::NodeType::Map);
    for ( auto node : base_node )
    {
        if ( node.first.IsScalar() )
        {
            const std::string& key = node.first.Scalar();
            if ( override_node[key] )
            {
                new_node[node.first.Scalar()] = Parser2::mergeYAML(
                        node.second, override_node[key]);
                continue;
            }
        }
        new_node[node.first.Scalar()] = node.second;
    }

    /* Add the mappings from 'override_node' not already in 'new_node' */
    for ( auto node : override_node )
    {
        if ( !node.first.IsScalar() || !new_node[node.first.Scalar()] )
        {
            new_node[node.first.Scalar()] = node.second;
        }
    }
    return YAML::Node(new_node);
}

void Parser2::log(const std::string& msg, bool print_error_msg)
{
    if ( print_error_msg )
    {
        std::cout << "\033[31m" // change terminal color to red
                  << "[Parser2] "
                  << msg
                  << "\033[0m" // restore terminal color
                  << std::endl;
    }
}

} // namespace yaml_common
} // namespace kelo
