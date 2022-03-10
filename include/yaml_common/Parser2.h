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

        template <typename T>
        static bool read(
                const YAML::Node& node,
                const std::string& key,
                T& value,
                bool print_error_msg = true);

        template <typename T>
        static bool read(
                const YAML::Node& node,
                T& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::Point2D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::Point3D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::XYTheta& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::Pose2D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::TransformMatrix2D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::TransformMatrix3D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::Box& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::LineSegment2D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::Polyline2D& value,
                bool print_error_msg = true);

        static bool read(
                const YAML::Node& node,
                geometry_common::Polygon2D& value,
                bool print_error_msg = true);

        template <typename T>
        static bool has(const YAML::Node& node, const std::string& key)
        {
            T dummy;
            return read(node, key, dummy, false);
        }

        template <typename T>
        static bool is(const YAML::Node& node)
        {
            T dummy;
            return read(node, dummy, false);
        }

        template <typename T>
        static T get(const YAML::Node& node, const std::string& key, T default_value)
        {
            T value;
            return Parser2::read(node, key, value, false) ? value : default_value;
        }

        template <typename T>
        static T get(const YAML::Node& node, T default_value)
        {
            T value;
            return Parser2::read(node, value, false) ? value : default_value;
        }

        static bool readFloats(
                const YAML::Node& node,
                const std::vector<std::string>& keys,
                std::vector<float>& values,
                bool print_error_msg = true);

        static bool performSanityChecks(
                const YAML::Node& node,
                const std::string& key,
                bool print_error_msg = true);

        static YAML::Node mergeYAML(
                const YAML::Node& default_node,
                const YAML::Node& override_node);

};

} // namespace yaml_common
} // namespace kelo
#endif // KELO_YAML_COMMON_PARSER_2_H
