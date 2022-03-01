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

#ifndef KELO_YAML_COMMON_PARSER_H
#define KELO_YAML_COMMON_PARSER_H

#include <geometry_common/pose_2d.h>
#include <yaml-cpp/yaml.h>

namespace kelo::yaml_common
{

/**
 * @brief A class with utility functions to query/parse YAML data into C++ types
 *
 */
class Parser
{
    public:
    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key is a Map data
     * structure
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if its a Map
     * @return bool True if there exists a key-value pair corresponding to
     * the input key and the value of this pair is a Map data structure
     */
    static bool hasMap(const YAML::Node& node, std::string key);

    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key is an integer. If
     * the key is an empty string, checks if the YAML node is itself an integer
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if its an integer
     * @return bool True if: \n
     * 1. There exists a key-value pair corresponding to the input key and the
     * value of this pair is an integer. \n
     * 2. The input key was an empty string and the input node is an integer
     */
    static bool hasInt(const YAML::Node& node, std::string key = "");

    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key is a double. If
     * the key is an empty string, checks if the YAML node is itself a double
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if its a double
     * @return bool True if: \n
     * 1. There exists a key-value pair corresponding to the input key and the
     * value of this pair is a double. \n
     * 2. The input key was an empty string and the input node is a double
     */
    static bool hasDouble(const YAML::Node& node, std::string key = "");

    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key is a bool. If
     * the key is an empty string, checks if the YAML node is itself a bool
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if its a bool
     * @return bool True if: \n
     * 1. There exists a key-value pair corresponding to the input key and the
     * value of this pair is a bool. \n
     * 2. The input key was an empty string and the input node is a bool
     */
    static bool hasBool(const YAML::Node& node, std::string key = "");

    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key is a string
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if its a string
     * @return bool True if there exists a key-value pair corresponding to
     * the input key and the value of this pair is a string
     */
    static bool hasString(const YAML::Node& node, std::string key);

    /**
     * @brief Checks if the YAML node represents 2D pose information
     *
     * @param node The YAML node to be checked
     * @return bool True if the node represents a 2D pose
     */
    static bool hasPose(const YAML::Node& node);

    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key represents 2D Pose
     * information
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if its a 2D Pose
     * @return bool True if there exists a key-value pair corresponding to
     * the input key and the value of this pair represents a 2D Pose
     */
    static bool hasPose(const YAML::Node& node, std::string key);

    /**
     * @brief Checks if a YAML Node contains a key-value pair corresponding
     * to the input key, and that the value of the input key represents Time
     * information
     *
     * @param node The YAML node to be checked
     * @param key The key, whose value should be checked if it represents time
     * information
     * @return bool True if there exists a key-value pair corresponding to
     * the input key and the value of this pair represents time information
     */
    static bool hasTime(const YAML::Node& node, std::string key);

    /**
     * @brief If a YAML Node contains a key-value pair corresponding to the
     * input key, the function tries to convert and return the value as an
     * integer. If the key is an empty string, the function tries to convert the
     * input node to an integer
     *
     * @param node The node to be parsed
     * @param key Optional, key to be checked in case the YAML node is a Map
     * data structure
     * @return int The parsed integer value if parsing was successful, else
     * returns 0
     */
    static int getInt(const YAML::Node& node, std::string key = "");

    /**
     * @brief If a YAML Node contains a key-value pair corresponding to the
     * input key, the function tries to convert and return the value as a
     * double. If the key is an empty string, the function tries to convert the
     * input node to a double
     *
     * @param node The node to be parsed
     * @param key Optional, key to be checked in case the YAML node is a Map
     * data structure
     * @return double The parsed double value if parsing was successful, else
     * returns 0.0
     */
    static double getDouble(const YAML::Node& node, std::string key = "");

    /**
     * @brief If a YAML Node contains a key-value pair corresponding to the
     * input key, the function tries to convert and return the value as a
     * bool. If the key is an empty string, the function tries to convert the
     * input node to a bool
     *
     * @param node The node to be parsed
     * @param key Optional, key to be checked in case the YAML node is a Map
     * data structure
     * @param defaultValue The default value to be returned in case parsing
     * fails
     * @return bool The parsed bool value if parsing was successful, else
     * returns the default value
     */
    static bool getBool(const YAML::Node& node, std::string key = "",
                            bool defaultValue = false);

    /**
     * @brief If a YAML Node contains a key-value pair corresponding to the
     * input key, the function tries to convert and return the value as a
     * string. If the key is an empty string, the function tries to convert the
     * input node to a string
     *
     * @param node The node to be parsed
     * @param key Optional, key to be checked in case the YAML node is a Map
     * data structure
     * @return std::string The parsed string value if parsing was successful,
     * else returns an empty string
     */
    static std::string getString(const YAML::Node& node,
                                     std::string key = "");

    /**
     * @brief Parse a YAML node into a 2D pose object
     *
     * @param node The YAML node to be parsed
     * @return geometry_common::Pose2d If parsing was successful, returns the
     * parsed 2D pose object, else returns a 'zero' 2D pose
     */
    static geometry_common::Pose2d getPose(const YAML::Node& node);

    /**
     * @brief Parse a value of a key-value pair inside a YAML node into a 2D
     * pose object
     *
     * @param node The YAML node to be parsed
     * @param key The key of the key-value pair that must be parsed as a 2D pose
     * object
     * @return geometry_common::Pose2d If parsing was successful, returns the
     * parsed 2D pose object, else returns a 'zero' 2D pose
     */
    static geometry_common::Pose2d getPose(const YAML::Node& node,
                                               std::string key);

    /**
     * @brief Parse a value of a key-value pair inside a YAML node into time
     * data
     *
     * @param node The YAML node to be parsed
     * @param key The key of the key-value pair that must be parsed as time data
     * @return long long If parsing was successful, returns the parsed time
     * data, else returns 0
     */
    static long long getTime(const YAML::Node& node, std::string key);

    /**
     * @brief Get the length of a list data structure represented as a YAML Node
     *
     * @param node The YAML node representing the list data structure
     * @return unsigned int If the node could be parsed as a list, returns the
     * length of the list, else returns 0
     */
    static unsigned int getLength(const YAML::Node& node);

    /**
     * @brief Get the length of a list data structure defined as the value of a
     * key-value pair in the input YAML node
     *
     * @param node The YAML node containing the list
     * @param key The key to access the list
     * @return unsigned int If the YAML data could be parsed as a list, returns
     * the length of the list, else returns 0
     */
    static unsigned int getLength(const YAML::Node& node, std::string key);

    /**
     * @brief Get a list of all the keys in the YAML node representing a Map
     * data structure
     *
     * @param node The YAML node to be parsed
     * @return std::vector<std::string> The list of keys
     */
    static std::vector<std::string> getAllKeys(const YAML::Node& node);

    /**
     * @brief Helper function to avoid bug of yaml-cpp when reading strings. \n
     * Bug was fixed, but fixed version not yet in all Linux distros.
     *
     * @param values The YAML node to be copied
     * @param yaml The object where the YAML data should be copied to
     * @param skipKeys List of keys to be skipped during copying of YAML data
     */
    static void copyYaml(
        const YAML::Node& values, YAML::Emitter& yaml,
        const std::vector<std::string>& skipKeys = std::vector<std::string>());

    /**
     * @brief Copy (and overwrite) values to target node, leave other keys in
     * target untouched. Only copies root keys of map types.
     *
     * @param values
     * @param target
     */
    static void copyYaml(const YAML::Node& values, YAML::Node& target);
};

} // namespace kelo::yaml_common

#endif // KELO_YAML_COMMON_PARSER_H
