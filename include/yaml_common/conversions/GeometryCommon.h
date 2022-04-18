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

#ifndef KELO_YAML_COMMON_CONVERSIONS_GEOMETRY_COMMON_H
#define KELO_YAML_COMMON_CONVERSIONS_GEOMETRY_COMMON_H

#include <yaml-cpp/yaml.h>

#include <geometry_common/Box.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/Pose2D.h>
#include <geometry_common/Circle.h>
#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/TransformMatrix3D.h>
#include <geometry_common/LineSegment2D.h>
#include <geometry_common/Polyline2D.h>
#include <geometry_common/Polygon2D.h>
#include <geometry_common/PointCloudProjector.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

namespace YAML
{

template<>
struct convert<kelo::geometry_common::Box>
{
    static Node encode(const kelo::geometry_common::Box& box);
    static bool decode(const Node& node, kelo::geometry_common::Box& box);
};

template<>
struct convert<kelo::geometry_common::Point2D>
{
    static Node encode(const kelo::geometry_common::Point2D& pt);
    static bool decode(const Node& node, kelo::geometry_common::Point2D& pt);
};

template<>
struct convert<kelo::geometry_common::Point3D>
{
    static Node encode(const kelo::geometry_common::Point3D& pt);
    static bool decode(const Node& node, kelo::geometry_common::Point3D& pt);
};

template<>
struct convert<kelo::geometry_common::XYTheta>
{
    static Node encode(const kelo::geometry_common::XYTheta& x_y_theta);
    static bool decode(const Node& node, kelo::geometry_common::XYTheta& x_y_theta);
};

template<>
struct convert<kelo::geometry_common::Pose2D>
{
    static Node encode(const kelo::geometry_common::Pose2D& pose);
    static bool decode(const Node& node, kelo::geometry_common::Pose2D& pose);
};

template<>
struct convert<kelo::geometry_common::Circle>
{
    static Node encode(const kelo::geometry_common::Circle& circle);
    static bool decode(const Node& node, kelo::geometry_common::Circle& circle);
};

template<>
struct convert<kelo::geometry_common::TransformMatrix2D>
{
    static Node encode(const kelo::geometry_common::TransformMatrix2D& tf_mat);
    static bool decode(const Node& node, kelo::geometry_common::TransformMatrix2D& tf_mat);
};

template<>
struct convert<kelo::geometry_common::TransformMatrix3D>
{
    static Node encode(const kelo::geometry_common::TransformMatrix3D& tf_mat);
    static bool decode(const Node& node, kelo::geometry_common::TransformMatrix3D& tf_mat);
};

template<>
struct convert<kelo::geometry_common::LineSegment2D>
{
    static Node encode(const kelo::geometry_common::LineSegment2D& line_segment);
    static bool decode(const Node& node, kelo::geometry_common::LineSegment2D& line_segment);
};

template<>
struct convert<kelo::geometry_common::Polyline2D>
{
    static Node encode(const kelo::geometry_common::Polyline2D& polyline);
    static bool decode(const Node& node, kelo::geometry_common::Polyline2D& polyline);
};

template<>
struct convert<kelo::geometry_common::Polygon2D>
{
    static Node encode(const kelo::geometry_common::Polygon2D& polygon);
    static bool decode(const Node& node, kelo::geometry_common::Polygon2D& polygon);
};

template<>
struct convert<kelo::PointCloudProjectorConfig>
{
    static Node encode(const kelo::PointCloudProjectorConfig& config);
    static bool decode(const Node& node, kelo::PointCloudProjectorConfig& config);
};

} // namespace YAML

#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif // KELO_YAML_COMMON_CONVERSIONS_GEOMETRY_COMMON_H
