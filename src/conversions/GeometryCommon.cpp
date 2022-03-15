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

#include <yaml_common/conversions/GeometryCommon.h>
#include <yaml_common/Parser2.h>

namespace YAML
{

Node convert<kelo::geometry_common::Box>::encode(
        const kelo::geometry_common::Box& box)
{
    Node node;
    node["min_x"] = box.min_x;
    node["max_x"] = box.max_x;
    node["min_y"] = box.min_y;
    node["max_y"] = box.max_y;
    node["min_z"] = box.min_z;
    node["max_z"] = box.max_z;
    return node;
}

bool convert<kelo::geometry_common::Box>::decode(
        const Node& node, kelo::geometry_common::Box& box)
{
    return ( kelo::yaml_common::Parser2::read<float>(node, "min_x", box.min_x) &&
             kelo::yaml_common::Parser2::read<float>(node, "max_x", box.max_x) &&
             kelo::yaml_common::Parser2::read<float>(node, "min_y", box.min_y) &&
             kelo::yaml_common::Parser2::read<float>(node, "max_y", box.max_y) &&
             kelo::yaml_common::Parser2::read<float>(node, "min_z", box.min_z) &&
             kelo::yaml_common::Parser2::read<float>(node, "max_z", box.max_z) );
}



Node convert<kelo::geometry_common::Point2D>::encode(
        const kelo::geometry_common::Point2D& pt)
{
    Node node;
    node["x"] = pt.x;
    node["y"] = pt.y;
    return node;
}

bool convert<kelo::geometry_common::Point2D>::decode(
        const Node& node, kelo::geometry_common::Point2D& pt)
{
    return ( kelo::yaml_common::Parser2::read<float>(node, "x", pt.x) &&
             kelo::yaml_common::Parser2::read<float>(node, "y", pt.y) );
}



Node convert<kelo::geometry_common::Point3D>::encode(
        const kelo::geometry_common::Point3D& pt)
{
    Node node;
    node["x"] = pt.x;
    node["y"] = pt.y;
    node["z"] = pt.z;
    return node;
}

bool convert<kelo::geometry_common::Point3D>::decode(
        const Node& node, kelo::geometry_common::Point3D& pt)
{
    return ( kelo::yaml_common::Parser2::read<float>(node, "x", pt.x) &&
             kelo::yaml_common::Parser2::read<float>(node, "y", pt.y) &&
             kelo::yaml_common::Parser2::read<float>(node, "z", pt.z) );
}



Node convert<kelo::geometry_common::XYTheta>::encode(
        const kelo::geometry_common::XYTheta& x_y_theta)
{
    Node node;
    node["x"] = x_y_theta.x;
    node["y"] = x_y_theta.y;
    node["theta"] = x_y_theta.theta;
    return node;
}

bool convert<kelo::geometry_common::XYTheta>::decode(
        const Node& node, kelo::geometry_common::XYTheta& x_y_theta)
{
    return ( kelo::yaml_common::Parser2::read<float>(node, "x", x_y_theta.x) &&
             kelo::yaml_common::Parser2::read<float>(node, "y", x_y_theta.y) &&
             kelo::yaml_common::Parser2::read<float>(node, "theta", x_y_theta.theta) );
}



Node convert<kelo::geometry_common::Pose2D>::encode(
        const kelo::geometry_common::Pose2D& pose)
{
    Node node;
    node["x"] = pose.x;
    node["y"] = pose.y;
    node["theta"] = pose.theta;
    return node;
}

bool convert<kelo::geometry_common::Pose2D>::decode(
        const Node& node, kelo::geometry_common::Pose2D& pose)
{
    kelo::geometry_common::XYTheta temp_value;
    if ( !kelo::yaml_common::Parser2::read<kelo::geometry_common::XYTheta>(
                node, temp_value, true) )
    {
        return false;
    }
    pose = kelo::geometry_common::Pose2D(temp_value); // does angle clipping
    return true;
}



Node convert<kelo::geometry_common::TransformMatrix2D>::encode(
        const kelo::geometry_common::TransformMatrix2D& tf_mat)
{
    Node node;
    node["x"] = tf_mat.x();
    node["y"] = tf_mat.y();
    node["theta"] = tf_mat.theta();
    return node;
}

bool convert<kelo::geometry_common::TransformMatrix2D>::decode(
        const Node& node, kelo::geometry_common::TransformMatrix2D& tf_mat)
{
    std::vector<std::string> keys{"x", "y", "theta"};
    std::vector<std::string> keys_quat{"x", "y", "qx", "qy", "qz", "qw"};
    std::vector<float> values;

    if ( kelo::yaml_common::Parser2::readFloats(node, keys, values, false) )
    {
        tf_mat.update(values[0], values[1], values[2]);
        return true;
    }

    values.clear();
    if ( kelo::yaml_common::Parser2::readFloats(node, keys_quat, values, false) )
    {
        tf_mat.update(values[0], values[1], values[2],
                      values[3], values[4], values[5]);
        return true;
    }

    std::cout << "Could not read TransformMatrix2D with either euler or quaternion format."
              << std::endl;
    return false;
}



Node convert<kelo::geometry_common::TransformMatrix3D>::encode(
        const kelo::geometry_common::TransformMatrix3D& tf_mat)
{
    Node node;
    node["x"] = tf_mat.x();
    node["y"] = tf_mat.y();
    node["z"] = tf_mat.z();
    node["roll"] = tf_mat.roll();
    node["pitch"] = tf_mat.pitch();
    node["yaw"] = tf_mat.yaw();
    return node;
}

bool convert<kelo::geometry_common::TransformMatrix3D>::decode(
        const Node& node, kelo::geometry_common::TransformMatrix3D& tf_mat)
{
    std::vector<std::string> keys{"x", "y", "z", "roll", "pitch", "yaw"};
    std::vector<std::string> keys_quat{"x", "y", "z", "qx", "qy", "qz", "qw"};
    std::vector<float> values;

    if ( kelo::yaml_common::Parser2::readFloats(node, keys, values, false) )
    {
        tf_mat.update(values[0], values[1], values[2],
                      values[3], values[4], values[5]);
        return true;
    }

    values.clear();
    if ( kelo::yaml_common::Parser2::readFloats(node, keys_quat, values, false) )
    {
        tf_mat.update(values[0], values[1], values[2],
                      values[3], values[4], values[5], values[6]);
        return true;
    }

    std::cout << "Could not read TransformMatrix3D with either euler or quaternion format."
              << std::endl;
    return false;
}



Node convert<kelo::geometry_common::LineSegment2D>::encode(
        const kelo::geometry_common::LineSegment2D& line_segment)
{
    Node node;
    node["start"] = line_segment.start;
    node["end"] = line_segment.end;
    return node;
}

bool convert<kelo::geometry_common::LineSegment2D>::decode(
        const Node& node, kelo::geometry_common::LineSegment2D& line_segment)
{
    return ( kelo::yaml_common::Parser2::read<kelo::geometry_common::Point2D>(
                 node, "start", line_segment.start) &&
             kelo::yaml_common::Parser2::read<kelo::geometry_common::Point2D>(
                 node, "end", line_segment.end) );
}



Node convert<kelo::geometry_common::Polyline2D>::encode(
        const kelo::geometry_common::Polyline2D& polyline)
{
    Node node;
    for ( size_t i = 0; i < polyline.size(); i++ )
    {
        node.push_back(polyline[i]);
    }
    return node;
}

bool convert<kelo::geometry_common::Polyline2D>::decode(
        const Node& node, kelo::geometry_common::Polyline2D& polyline)
{
    return kelo::yaml_common::Parser2::read<std::vector<kelo::geometry_common::Point2D>>(
            node, polyline.vertices);
}



Node convert<kelo::geometry_common::Polygon2D>::encode(
        const kelo::geometry_common::Polygon2D& polygon)
{
    Node node;
    for ( size_t i = 0; i < polygon.size(); i++ )
    {
        node.push_back(polygon[i]);
    }
    return node;
}

bool convert<kelo::geometry_common::Polygon2D>::decode(
        const Node& node, kelo::geometry_common::Polygon2D& polygon)
{
    return kelo::yaml_common::Parser2::read<std::vector<kelo::geometry_common::Point2D>>(
            node, polygon.vertices);
}



Node convert<kelo::PointCloudProjectorConfig>::encode(
        const kelo::PointCloudProjectorConfig& config)
{
    Node node;
    node["transform"] = config.tf_mat;
    node["angle_min"] = config.angle_min;
    node["angle_max"] = config.angle_max;
    node["passthrough_min_z"] = config.passthrough_min_z;
    node["passthrough_max_z"] = config.passthrough_max_z;
    node["radial_dist_min"] = config.radial_dist_min;
    node["radial_dist_max"] = config.radial_dist_max;
    node["angle_increment"] = config.angle_increment;
    return node;
}

bool convert<kelo::PointCloudProjectorConfig>::decode(
        const Node& node, kelo::PointCloudProjectorConfig& config)
{
    kelo::yaml_common::Parser2::read<kelo::geometry_common::TransformMatrix3D>(
            node, "transform", config.tf_mat, false); // optional
    return ( kelo::yaml_common::Parser2::read<float>(
                 node, "angle_min", config.angle_min) &&
             kelo::yaml_common::Parser2::read<float>(
                 node, "angle_max", config.angle_max) &&
             kelo::yaml_common::Parser2::read<float>(
                 node, "passthrough_min_z", config.passthrough_min_z) &&
             kelo::yaml_common::Parser2::read<float>(
                 node, "passthrough_max_z", config.passthrough_max_z) &&
             kelo::yaml_common::Parser2::read<float>(
                 node, "radial_dist_min", config.radial_dist_min) &&
             kelo::yaml_common::Parser2::read<float>(
                 node, "radial_dist_max", config.radial_dist_max) &&
             kelo::yaml_common::Parser2::read<float>(
                 node, "angle_increment", config.angle_increment) );
    return true;
}

} // namespace YAML
