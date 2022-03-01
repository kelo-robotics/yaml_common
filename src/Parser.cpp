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

#include "yaml_common/Parser.h"

namespace kelo::yaml_common
{

bool Parser::yamlHasMap(const YAML::Node& node, std::string key)
{
    if (!node.IsMap() || !node[key])
        return false;

    return node[key].IsMap();
}

bool Parser::yamlHasInt(const YAML::Node& node, std::string key)
{
    try
    {
        if (key.empty())
            int i = node.as<int>();
        else
            int i = node[key].as<int>();
        return true;
    }
    catch (YAML::Exception&)
    {
    }

    return false;
}

bool Parser::yamlHasDouble(const YAML::Node& node, std::string key)
{
    try
    {
        if (key.empty())
            double d = node.as<double>();
        else
            double d = node[key].as<double>();
        return true;
    }
    catch (YAML::Exception&)
    {
    }

    return false;
}

bool Parser::yamlHasString(const YAML::Node& node, std::string key)
{
    if (!node.IsMap() || !node[key]) // TODO check if correct
        return false;

    return node[key].IsScalar();
}

bool Parser::yamlHasBool(const YAML::Node& node, std::string key)
{
    try
    {
        if (key.empty())
            bool bc = node.as<bool>();
        else
            bool bc = node[key].as<bool>();
        return true;
    }
    catch (YAML::Exception&)
    {
    }

    return false;
}

int Parser::yamlGetInt(const YAML::Node& node, std::string key)
{
    int i = 0;
    try
    {
        if (key.empty())
            i = node.as<int>();
        else
            i = node[key].as<int>();
    }
    catch (YAML::Exception&)
    {
    }

    return i;
}

double Parser::yamlGetDouble(const YAML::Node& node, std::string key)
{
    double d = 0;
    try
    {
        if (key.empty())
            d = node.as<double>();
        else
            d = node[key].as<double>();
    }
    catch (YAML::Exception&)
    {
    }

    return d;
}

std::string Parser::yamlGetString(const YAML::Node& node, std::string key)
{
    try
    {
        if (key.empty())
        {
            return node.as<std::string>();
        }
        else
        {
            return node[key].as<std::string>();
        }
    }
    catch (YAML::Exception&)
    {
    }

    return "";
}

bool Parser::yamlGetBool(const YAML::Node& node, std::string key,
                         bool defaultValue)
{
    bool b = defaultValue;
    try
    {
        if (key.empty())
            b = node.as<bool>();
        else
            b = node[key].as<bool>();
    }
    catch (YAML::Exception&)
    {
    }

    return b;
}

unsigned int Parser::yamlGetLength(const YAML::Node& node, std::string key)
{
    if (!node || !node[key] || !node[key].IsSequence())
        return 0;

    return node[key].size();
}

unsigned int Parser::yamlGetLength(const YAML::Node& node)
{
    if (!node || !node.IsSequence())
        return 0;

    return node.size();
}

std::vector<std::string> Parser::getAllKeys(const YAML::Node& node)
{
    std::vector<std::string> keys;
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
        keys.push_back(it->first.as<std::string>());
    }
    return keys;
}

bool Parser::yamlHasPose(const YAML::Node& node, std::string key)
{
    return (node[key] && node[key].IsSequence() && node[key].size() == 3 &&
            node[key][0].IsScalar() && node[key][1].IsScalar() &&
            node[key][2].IsScalar());
}

bool Parser::yamlHasPose(const YAML::Node& node)
{
    return (node && node.IsSequence() && node.size() == 3 &&
            node[0].IsScalar() && node[1].IsScalar() && node[2].IsScalar());
}

geometry_common::Pose2d Parser::yamlGetPose(const YAML::Node& node,
                                            std::string key)
{
    if (!yamlHasPose(node, key))
        return geometry_common::Pose2d();

    const YAML::Node& p = node[key];
    return geometry_common::Pose2d(p[0].as<double>(), p[1].as<double>(),
                                   p[2].as<double>());
}

geometry_common::Pose2d Parser::yamlGetPose(const YAML::Node& node)
{
    if (!yamlHasPose(node))
        return geometry_common::Pose2d();

    return geometry_common::Pose2d(node[0].as<double>(), node[1].as<double>(),
                                   node[2].as<double>());
}

bool Parser::yamlHasTime(const YAML::Node& node, std::string key)
{
    return (node[key] && node[key].IsMap() && yamlHasInt(node[key], "high") &&
            yamlHasInt(node[key], "low"));
}

long long Parser::yamlGetTime(const YAML::Node& node, std::string key)
{
    if (!yamlHasTime(node, key))
        return 0;

    unsigned int low = (unsigned int)yamlGetInt(node[key], "low");
    unsigned int high = (unsigned int)yamlGetInt(node[key], "high");

    return (((long long)high) << 32) + low;
}

void Parser::copyYaml(const YAML::Node& values, YAML::Emitter& yaml,
                      const std::vector<std::string>& skipKeys)
{
    if (values.IsMap())
    {
        for (YAML::const_iterator it = values.begin(); it != values.end();
        ++it)
        {
            std::string key = it->first.as<std::string>();
            bool skip = false;
            for (unsigned int i = 0; i < skipKeys.size() && !skip; i++)
                if (key == skipKeys[i])
                    skip = true;

            if (!skip)
            {
                if (it->second.IsScalar())
                {
                    if (it->second.Tag() == "!")
                    { // loaded as string in yaml-cpp
                        yaml << YAML::Key << key << YAML::Value
                             << it->second.as<std::string>();
                    }
                    else
                    {
                        if (yamlHasInt(it->second))
                            yaml << YAML::Key << key << YAML::Value
                                 << yamlGetInt(it->second);
                        else if (yamlHasDouble(it->second))
                            yaml << YAML::Key << key << YAML::Value
                                 << yamlGetDouble(it->second);
                        else if (yamlHasBool(it->second))
                            yaml << YAML::Key << key << YAML::Value
                                 << yamlGetBool(it->second);
                        else
                            yaml << YAML::Key << key << YAML::Value
                                 << yamlGetString(it->second);
                    }
                }
                else if (it->second.IsMap())
                {
                    yaml << YAML::Key << key;
                    yaml << YAML::BeginMap;
                    copyYaml(it->second, yaml);
                    yaml << YAML::EndMap;
                }
                else if (it->second.IsSequence())
                {
                    yaml << YAML::Key << key;
                    yaml << YAML::BeginSeq;
                    copyYaml(it->second, yaml);
                    yaml << YAML::EndSeq;
                }
            }
        }
    }
    else if (values.IsSequence())
    {
        for (YAML::const_iterator it = values.begin(); it != values.end();
        ++it)
        {
            if (it->IsScalar())
            {
                if (it->Tag() == "!")
                { // loaded as string in yaml-cpp
                    yaml << it->as<std::string>();
                }
                else
                {
                    if (yamlHasInt(*it))
                        yaml << yamlGetInt(*it);
                    else if (yamlHasDouble(*it))
                        yaml << yamlGetDouble(*it);
                    else if (yamlHasBool(*it))
                        yaml << yamlGetBool(*it);
                    else
                        yaml << yamlGetString(*it);
                }
            }
            else if (it->IsMap())
            {
                yaml << YAML::BeginMap;
                copyYaml(*it, yaml);
                yaml << YAML::EndMap;
            }
            else if (it->IsSequence())
            {
                yaml << YAML::BeginSeq;
                copyYaml(*it, yaml);
                yaml << YAML::EndSeq;
            }
        }
    }
}

void Parser::copyYaml(const YAML::Node& values, YAML::Node& target)
{
    if (!values.IsMap() || (!target.IsNull() && !target.IsMap()))
        return;

    for (YAML::const_iterator it = values.begin(); it != values.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        target[key] = it->second;
    }
}

} // namespace kelo::yaml_common
