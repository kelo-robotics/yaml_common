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
    static Node encode(const kelo::geometry_common::Box& pt);
    static bool decode(const Node& node, kelo::geometry_common::Box& pt);
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
    static Node encode(const kelo::geometry_common::XYTheta& pt);
    static bool decode(const Node& node, kelo::geometry_common::XYTheta& pt);
};

template<>
struct convert<kelo::geometry_common::Pose2D>
{
    static Node encode(const kelo::geometry_common::Pose2D& pt);
    static bool decode(const Node& node, kelo::geometry_common::Pose2D& pt);
};

template<>
struct convert<kelo::geometry_common::TransformMatrix2D>
{
    static Node encode(const kelo::geometry_common::TransformMatrix2D& pt);
    static bool decode(const Node& node, kelo::geometry_common::TransformMatrix2D& pt);
};

template<>
struct convert<kelo::geometry_common::TransformMatrix3D>
{
    static Node encode(const kelo::geometry_common::TransformMatrix3D& pt);
    static bool decode(const Node& node, kelo::geometry_common::TransformMatrix3D& pt);
};

template<>
struct convert<kelo::geometry_common::LineSegment2D>
{
    static Node encode(const kelo::geometry_common::LineSegment2D& pt);
    static bool decode(const Node& node, kelo::geometry_common::LineSegment2D& pt);
};

template<>
struct convert<kelo::geometry_common::Polyline2D>
{
    static Node encode(const kelo::geometry_common::Polyline2D& pt);
    static bool decode(const Node& node, kelo::geometry_common::Polyline2D& pt);
};

template<>
struct convert<kelo::geometry_common::Polygon2D>
{
    static Node encode(const kelo::geometry_common::Polygon2D& pt);
    static bool decode(const Node& node, kelo::geometry_common::Polygon2D& pt);
};

template<>
struct convert<kelo::PointCloudProjectorConfig>
{
    static Node encode(const kelo::PointCloudProjectorConfig& pt);
    static bool decode(const Node& node, kelo::PointCloudProjectorConfig& pt);
};

} // namespace YAML

#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif // KELO_YAML_COMMON_CONVERSIONS_GEOMETRY_COMMON_H
