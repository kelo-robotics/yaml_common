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

#include <geometry_common/Box.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/Pose2D.h>
#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/TransformMatrix3D.h>
#include <geometry_common/LineSegment2D.h>
#include <geometry_common/Polyline2D.h>
#include <geometry_common/Polygon2D.h>

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
         * @brief Read value of `node`[`key`] into `value` when possible
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
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` into `value` when possible
         *
         * @tparam T type of value to be read. This is used for generic c++
         * datatypes (int, float, double, unsigned int, bool, std::string)
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
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as Point2D object
         *
         * @param node YAML node that needs to be parsed
         * @param value Point2D variable to which the parsed values should be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::Point2D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as Point3D object
         *
         * @param node YAML node that needs to be parsed
         * @param value Point3D variable to which the parsed values should be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::Point3D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as XYTheta object
         *
         * @param node YAML node that needs to be parsed
         * @param value XYTheta variable to which the parsed values should be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::XYTheta& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as Pose2D object
         *
         * @param node YAML node that needs to be parsed
         * @param value Pose2D variable to which the parsed values should be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::Pose2D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as TransformMatrix2D object
         *
         * @param node YAML node that needs to be parsed
         * @param value TransformMatrix2D variable to which the parsed values
         * should be assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::TransformMatrix2D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as TransformMatrix3D object
         *
         * @param node YAML node that needs to be parsed
         * @param value TransformMatrix3D variable to which the parsed values
         * should be assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::TransformMatrix3D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as Box object
         *
         * @param node YAML node that needs to be parsed
         * @param value Box variable to which the parsed values should be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::Box& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as LineSegment2D object
         *
         * @param node YAML node that needs to be parsed
         * @param value LineSegment2D variable to which the parsed values should
         * be assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::LineSegment2D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as Polyline2D object
         *
         * @param node YAML node that needs to be parsed
         * @param value Polyline2D variable to which the parsed values should
         * be assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::Polyline2D& value,
                bool print_error_msg = true);

        /**
         * @brief Read value of `node` as Polygon2D object
         *
         * @param node YAML node that needs to be parsed
         * @param value Polygon2D variable to which the parsed values should be
         * assigned
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return bool success in reading the value
         */
        static bool read(
                const YAML::Node& node,
                geometry_common::Polygon2D& value,
                bool print_error_msg = true);

        /**
         * @brief Check if `node` contains `key` as one of its key-value pairs
         * and the value of that key can be read as `T` datatype
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be checked
         * @param key key that needs to be checked in `node`
         * @return true if `key` exists in `node` map and the value can be read
         * as type `T`
         */
        template <typename T>
        static bool has(const YAML::Node& node, const std::string& key)
        {
            T dummy;
            return read(node, key, dummy, false);
        }

        /**
         * @brief Check if `node` can be read as `T` datatype
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be checked
         * @return true if `node` can be read as type `T`
         */
        template <typename T>
        static bool is(const YAML::Node& node)
        {
            T dummy;
            return read(node, dummy, false);
        }

        /**
         * @brief Parse and return `node`[`key`] as `T` datatype if possible,
         * otherwise return the `default_value`.
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
        static T get(const YAML::Node& node, const std::string& key, T default_value)
        {
            T value;
            return Parser2::read(node, key, value, false) ? value : default_value;
        }

        /**
         * @brief Parse and return `node` as `T` datatype if possible,
         * otherwise return the `default_value`.
         *
         * @tparam T type of value to be read
         * @param node YAML node that needs to be parsed
         * @param default_value value that needs to be returned if parsing fails
         * for any reason
         * @return parsed value of node if possible, otherwise default_value
         */
        template <typename T>
        static T get(const YAML::Node& node, T default_value)
        {
            T value;
            return Parser2::read(node, value, false) ? value : default_value;
        }

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
         * @brief Perform the following sanity checks
         * 1. `key` is not an empty string
         * 2. `node` is of YAML map type
         * 3. `node` contains a key named `key`
         *
         * @param node YAML map node
         * @param key key to be checked
         * @param print_error_msg decides whether to print error message when
         * parsing is unsuccessful.
         * @return success Only true when all sanity checks pass
         */
        static bool performSanityChecks(
                const YAML::Node& node,
                const std::string& key,
                bool print_error_msg = true);

        /**
         * @brief Merge two YAML map nodes into one. Exclusive key-value pairs
         * from the input nodes are copied as is. For conflicting keys,
         * values from `override_node` are used.
         *
         * @param default_node YAML map node used as base
         * @param override_node YAML map node whose values will be used when a
         * key is present in both nodes
         * @return YAML map node that is a merged map of both input nodes
         */
        static YAML::Node mergeYAML(
                const YAML::Node& default_node,
                const YAML::Node& override_node);

};

} // namespace yaml_common
} // namespace kelo
#endif // KELO_YAML_COMMON_PARSER_2_H
