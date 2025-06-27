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

#ifndef KELO_YAML_COMMON_PARSER_2_H
#define KELO_YAML_COMMON_PARSER_2_H

#include <yaml-cpp/yaml.h>

#ifdef USE_GEOMETRY_COMMON
#include <yaml_common/conversions/GeometryCommon.h>
#endif // USE_GEOMETRY_COMMON

namespace kelo
{
namespace yaml_common
{

/**
 * @brief A class with utility functions to query/parse YAML data into C++ types
 *
 */
class Parser2
{
    public:

        /**
         * @brief Load a .yaml file from disk with error checking
         *
         * @param abs_file_path Absolute path of .yaml file
         * @param node YAML node where the loaded file's content will be read to
         * @param print_error_msg decides whether to print error message when
         * loading is unsuccessful.
         * @return bool success in loading the file
         */
        static bool loadFile(
                const std::string& abs_file_path,
                YAML::Node& node,
                bool print_error_msg = true);

        /**
         * @brief Load a string with error checking
         *
         * @param s string to be parsed
         * @param node YAML node where the string's content will be read to
         * @param print_error_msg decides whether to print error message when
         * loading is unsuccessful.
         * @return bool success in loading the string
         */
        static bool loadString(
                const std::string& s,
                YAML::Node& node,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node`[`key`] into `value` when possible
         *
         * example:
         * \code
         *     bool success = Parser2::read<int>(node, "key", your_int_variable);
         *     bool success = Parser2::read<std::vector<int>>(node, "key", your_int_vec_variable);
         *     bool success = Parser2::read<Point2D>(node, "key", your_point_variable);
         *     bool success = Parser2::read<std::vector<Point2D>>(node, "key", your_pt_vec_variable);
         * \endcode
         *
         * @tparam T type of value to be read
         * @param node YAML node map that needs to be parsed
         * @param key key to be checked in node
         * @param value variable to which the parsed values should be assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        template <typename T>
        static bool read(
                const YAML::Node& node,
                const std::string& key,
                T& value,
                bool print_error_msg = true)
        {
            if ( !Parser2::hasKey(node, key, print_error_msg) )
            {
                return false;
            }
            if ( !read(node[key], value, print_error_msg) )
            {
                std::stringstream msg;
                msg << "Could not read YAML::Node with key " << key;
                Parser2::log(msg.str(), print_error_msg);
                return false;
            }
            return true;
        }

        /**
         * @brief Read value of `node` into `value` when possible.
         *
         * example:
         * \code
         *     bool success = Parser2::read<int>(node, your_int_variable);
         *     bool success = Parser2::read<std::vector<int>>(node, your_int_vec_variable);
         *     bool success = Parser2::read<Point2D>(node, your_point_variable);
         *     bool success = Parser2::read<std::vector<Point2D>>(node, your_pt_vec_variable);
         * \endcode
         *
         * @tparam T type of value to be read.
         * @param node YAML node that needs to be parsed
         * @param value variable to which the parsed values should be assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        template <typename T>
        static bool read(
                const YAML::Node& node,
                T& value,
                bool print_error_msg = true)
        {
            try
            {
                value = node.as<T>();
            }
            catch ( YAML::Exception& )
            {
                Parser2::log("Could not read value of YAML::Node", print_error_msg);
                return false;
            }

            return true;
        }

        /**
         * @brief Check if `node` contains `key` as one of its key-value pairs
         * and the value of that key can be read as `T` datatype
         *
         * example:
         * \code
         *     bool node_has_int = Parser2::has<int>(node, "key");
         *     bool node_has_point = Parser2::has<Point2D>(node, "key");
         * \endcode
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be checked
         * @param key key that needs to be checked in `node`
         * @return true if `key` exists in `node` map and the value can be read
         * as type `T`
         */
        template <typename T>
        static bool has(
                const YAML::Node& node,
                const std::string& key)
        {
            T dummy;
            return read(node, key, dummy, false);
        }

        /**
         * @brief Check if `node` contains `key` or not. Additionally it also
         * checks for the following two things
         * 1. `key` is not an empty string
         * 2. `node` is of YAML map type
         *
         * @param node YAML map node
         * @param key key to be checked
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return true when node contains key
         */
        static bool hasKey(
                const YAML::Node& node,
                const std::string& key,
                bool print_error_msg = true);

        /**
         * @brief Check if `node` can be read as `T` datatype
         *
         * example:
         * \code
         *     bool is_node_int = Parser2::is<int>(node);
         *     bool is_node_point = Parser2::is<Point2D>(node);
         * \endcode
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be checked
         * @return true if `node` can be read as type `T`
         */
        template <typename T>
        static bool is(
                const YAML::Node& node)
        {
            T dummy;
            return read(node, dummy, false);
        }

        /**
         * @brief Parse and return `node`[`key`] as `T` datatype if possible,
         * otherwise return the `default_value`.
         *
         * example:
         * \code
         *     int your_int_variable = Parser2::get<int>(node, "key", default_int_value);
         *     Point2D your_point_variable = Parser2::get<Point2D>(node, "key", default_point_value);
         * \endcode
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be parsed
         * @param key key that needs to be read in `node`
         * @param default_value value that needs to be returned if parsing fails
         * for any reason
         * @return parsed value from node[key] if possible, otherwise
         * default_value
         */
        template <typename T>
        static T get(
                const YAML::Node& node,
                const std::string& key,
                const T& default_value)
        {
            T value;
            return Parser2::read(node, key, value, false) ? value : default_value;
        }

        /**
         * @brief Parse and return `node` as `T` datatype if possible,
         * otherwise return the `default_value`.
         *
         * example:
         * \code
         *     int your_int_variable = Parser2::get<int>(node, default_int_value);
         *     Point2D your_point_variable = Parser2::get<Point2D>(node, default_point_value);
         * \endcode
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be parsed
         * @param default_value value that needs to be returned if parsing fails
         * for any reason
         * @return parsed value of node if possible, otherwise default_value
         */
        template <typename T>
        static T get(
                const YAML::Node& node,
                const T& default_value)
        {
            T value;
            return Parser2::read(node, value, false) ? value : default_value;
        }

        /**
         * @brief Parse an ordered list of all the keys present in a YAML map
         * 
         * @param node YAML map node that needs to be parsed
         * @param keys vector of strings where the keys will be read
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return success Only true when the yaml node is a map and all the 
         * keys where parsed successfully
         */
        static bool readAllKeys(
            const YAML::Node& node,
            std::vector<std::string>& keys,
            bool print_error_msg = true);

        /**
         * @brief Given a vector of keys, parse their values from a YAML map
         * node.
         *
         * @param node YAML map node that needs to be parsed
         * @param keys vector of keys whose values need to be read
         * @param values vector of floats where the read values need to be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return success Only true when all the keys where parsed successfully
         */
        static bool readFloats(
                const YAML::Node& node,
                const std::vector<std::string>& keys,
                std::vector<float>& values,
                bool print_error_msg = true);

        /**
         * @brief Merge two YAML map nodes into one. Exclusive key-value pairs
         * from the input nodes are copied as is. For conflicting keys,
         * values from `override_node` are used.
         *
         * @param base_node YAML map node used as base
         * @param override_node YAML map node whose values will be used when a
         * key is present in both nodes
         * @return YAML map node that is a merged map of both input nodes
         */
        static YAML::Node mergeYAML(
                const YAML::Node& base_node,
                const YAML::Node& override_node);

        /**
         * @brief Loosely check if two nodes are equal or not.
         * *NOTE*: Casts to std::string for scalar. Implies that 5.0 == 5 and "5" == 5
         * *NOTE*: for map type, if keys are not strings, will return false
         *
         * @param n1 first node
         * @param n2 second node
         *
         * @return true if n1 == n2, false otherwise
         */
        static bool isEqual(
                const YAML::Node& n1,
                const YAML::Node& n2);

    protected:

        /**
         * @brief Print error message to `std::cout` if `print_error_msg` is
         * true in red colored font
         *
         * @param msg Message that needs to be printed
         * @param print_error_msg decides whether to print error message or not.
         * If it is false, nothing gets printed.
         */
        static void log(
                const std::string& msg,
                bool print_error_msg = true);

};

} // namespace yaml_common
} // namespace kelo

#endif // KELO_YAML_COMMON_PARSER_2_H
