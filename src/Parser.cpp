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

bool Parser::hasMap(const YAML::Node& node, std::string key)
{
    if (!node.IsMap() || !node[key])
        return false;

    return node[key].IsMap();
}

bool Parser::hasInt(const YAML::Node& node, std::string key)
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

bool Parser::hasDouble(const YAML::Node& node, std::string key)
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

bool Parser::hasString(const YAML::Node& node, std::string key)
{
    if (!node.IsMap() || !node[key]) // TODO check if correct
        return false;

    return node[key].IsScalar();
}

bool Parser::hasBool(const YAML::Node& node, std::string key)
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

int Parser::getInt(const YAML::Node& node, std::string key)
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

double Parser::getDouble(const YAML::Node& node, std::string key)
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

std::string Parser::getString(const YAML::Node& node, std::string key)
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

bool Parser::getBool(const YAML::Node& node, std::string key,
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

unsigned int Parser::getLength(const YAML::Node& node, std::string key)
{
    if (!node || !node[key] || !node[key].IsSequence())
        return 0;

    return node[key].size();
}

unsigned int Parser::getLength(const YAML::Node& node)
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

bool Parser::hasPose(const YAML::Node& node, std::string key)
{
    return (node[key] && node[key].IsSequence() && node[key].size() == 3 &&
            node[key][0].IsScalar() && node[key][1].IsScalar() &&
            node[key][2].IsScalar());
}

bool Parser::hasPose(const YAML::Node& node)
{
    return (node && node.IsSequence() && node.size() == 3 &&
            node[0].IsScalar() && node[1].IsScalar() && node[2].IsScalar());
}

geometry_common::Pose2d Parser::getPose(const YAML::Node& node,
                                            std::string key)
{
    if (!hasPose(node, key))
        return geometry_common::Pose2d();

    const YAML::Node& p = node[key];
    return geometry_common::Pose2d(p[0].as<double>(), p[1].as<double>(),
                                   p[2].as<double>());
}

geometry_common::Pose2d Parser::getPose(const YAML::Node& node)
{
    if (!hasPose(node))
        return geometry_common::Pose2d();

    return geometry_common::Pose2d(node[0].as<double>(), node[1].as<double>(),
                                   node[2].as<double>());
}

bool Parser::hasTime(const YAML::Node& node, std::string key)
{
    return (node[key] && node[key].IsMap() && hasInt(node[key], "high") &&
            hasInt(node[key], "low"));
}

long long Parser::getTime(const YAML::Node& node, std::string key)
{
    if (!hasTime(node, key))
        return 0;

    unsigned int low = (unsigned int)getInt(node[key], "low");
    unsigned int high = (unsigned int)getInt(node[key], "high");

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
                        if (hasInt(it->second))
                            yaml << YAML::Key << key << YAML::Value
                                 << getInt(it->second);
                        else if (hasDouble(it->second))
                            yaml << YAML::Key << key << YAML::Value
                                 << getDouble(it->second);
                        else if (hasBool(it->second))
                            yaml << YAML::Key << key << YAML::Value
                                 << getBool(it->second);
                        else
                            yaml << YAML::Key << key << YAML::Value
                                 << getString(it->second);
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
                    if (hasInt(*it))
                        yaml << getInt(*it);
                    else if (hasDouble(*it))
                        yaml << getDouble(*it);
                    else if (hasBool(*it))
                        yaml << getBool(*it);
                    else
                        yaml << getString(*it);
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
