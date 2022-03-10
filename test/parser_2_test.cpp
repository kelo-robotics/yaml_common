#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <yaml_common/Parser2.h>

#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/Pose2D.h>

using Parser = kelo::yaml_common::Parser2;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Point3D;
using kelo::geometry_common::XYTheta;
using kelo::geometry_common::Pose2D;

TEST(Parser2Test, generic_datatypes)
{
    YAML::Node node;
    node["i"] = 5;
    node["f"] = 5.5f;
    node["d"] = 5.5;
    node["u"] = 5u;
    node["b"] = true;
    node["s"] = "abc";

    int test_int = 2;
    EXPECT_EQ(Parser::read<int>(node, "i2", test_int), false); // read from map but with incorrect key
    EXPECT_EQ(test_int, 2); // check if value is not overwritten
    EXPECT_EQ(Parser::read<int>(node, "i", test_int), true); // read from map with correct key
    EXPECT_EQ(test_int, 5); // check if value is read correctly
    test_int = 2;
    EXPECT_EQ(Parser::read<int>(node["s"], test_int), false); // read value directly from node with another key with value string
    EXPECT_EQ(test_int, 2); // check if value is not overwritten
    EXPECT_EQ(Parser::read<int>(node["i2"], test_int), false); // read value from an invalid/empty node
    EXPECT_EQ(test_int, 2); // check if value is not overwritten
    EXPECT_EQ(Parser::read<int>(node["i"], test_int), true); // read value directly from node with correct key
    EXPECT_EQ(test_int, 5); // check if value is read correctly
    EXPECT_EQ(Parser::has<int>(node, "i"), true);
    EXPECT_EQ(Parser::has<int>(node, "i2"), false);
    EXPECT_EQ(Parser::is<int>(node["i"]), true);
    EXPECT_EQ(Parser::is<int>(node["s"]), false);
    EXPECT_EQ(Parser::get<int>(node, "i", 0), 5);
    EXPECT_EQ(Parser::get<int>(node, "i2", 0), 0);
    EXPECT_EQ(Parser::get<int>(node["i"], 0), 5);
    EXPECT_EQ(Parser::get<int>(node["s"], 0), 0);

    float test_float = 2.0f;
    EXPECT_EQ(Parser::read<float>(node, "f2", test_float), false); // read from map but with incorrect key
    EXPECT_NEAR(test_float, 2.0f, 1e-9f); // check if value is not overwritten
    EXPECT_EQ(Parser::read<float>(node, "f", test_float), true); // read from map with correct key
    EXPECT_NEAR(test_float, 5.5f, 1e-9f); // check if value is read correctly
    test_float = 2.0f;
    EXPECT_EQ(Parser::read<float>(node["s"], test_float), false); // read value directly from node with another key with value string
    EXPECT_NEAR(test_float, 2.0f, 1e-9f); // check if value is not overwritten
    EXPECT_EQ(Parser::read<float>(node["f2"], test_float), false); // read value from an invalid/empty node
    EXPECT_NEAR(test_float, 2.0f, 1e-9f); // check if value is not overwritten
    EXPECT_EQ(Parser::read<float>(node["f"], test_float), true); // read value directly from node with correct key
    EXPECT_NEAR(test_float, 5.5f, 1e-9f); // check if value is read correctly
    EXPECT_EQ(Parser::has<float>(node, "f"), true);
    EXPECT_EQ(Parser::has<float>(node, "f2"), false);
    EXPECT_EQ(Parser::is<float>(node["f"]), true);
    EXPECT_EQ(Parser::is<float>(node["s"]), false);
    EXPECT_NEAR(Parser::get<float>(node, "f", 0.0f), 5.5f, 1e-9f);
    EXPECT_NEAR(Parser::get<float>(node, "f2", 0.0f), 0.0f, 1e-9f);
    EXPECT_NEAR(Parser::get<float>(node["f"], 0.0f), 5.5f, 1e-9f);
    EXPECT_NEAR(Parser::get<float>(node["s"], 0.0f), 0.0f, 1e-9f);

    double test_double = 2.0;
    EXPECT_EQ(Parser::read<double>(node, "d2", test_double), false); // read from map but with incorrect key
    EXPECT_NEAR(test_double, 2.0, 1e-9); // check if value is not overwritten
    EXPECT_EQ(Parser::read<double>(node, "d", test_double), true); // read from map with correct key
    EXPECT_NEAR(test_double, 5.5, 1e-9); // check if value is read correctly
    test_double = 2.0;
    EXPECT_EQ(Parser::read<double>(node["s"], test_double), false); // read value directly from node with another key with value string
    EXPECT_NEAR(test_double, 2.0, 1e-9); // check if value is not overwritten
    EXPECT_EQ(Parser::read<double>(node["d2"], test_double), false); // read value from an invalid/empty node
    EXPECT_NEAR(test_double, 2.0, 1e-9); // check if value is not overwritten
    EXPECT_EQ(Parser::read<double>(node["d"], test_double), true); // read value directly from node with correct key
    EXPECT_NEAR(test_double, 5.5, 1e-9); // check if value is read correctly
    EXPECT_EQ(Parser::has<double>(node, "d"), true);
    EXPECT_EQ(Parser::has<double>(node, "d2"), false);
    EXPECT_EQ(Parser::is<double>(node["d"]), true);
    EXPECT_EQ(Parser::is<double>(node["s"]), false);
    EXPECT_NEAR(Parser::get<double>(node, "d", 0.0), 5.5, 1e-9);
    EXPECT_NEAR(Parser::get<double>(node, "d2", 0.0), 0.0, 1e-9);
    EXPECT_NEAR(Parser::get<double>(node["d"], 0.0), 5.5, 1e-9);
    EXPECT_NEAR(Parser::get<double>(node["s"], 0.0), 0.0, 1e-9);

    unsigned int test_unsigned_int = 2;
    EXPECT_EQ(Parser::read<unsigned int>(node, "u2", test_unsigned_int), false); // read from map but with incorrect key
    EXPECT_EQ(test_unsigned_int, 2u); // check if value is not overwritten
    EXPECT_EQ(Parser::read<unsigned int>(node, "u", test_unsigned_int), true); // read from map with correct key
    EXPECT_EQ(test_unsigned_int, 5u); // check if value is read correctly
    test_unsigned_int = 2;
    EXPECT_EQ(Parser::read<unsigned int>(node["s"], test_unsigned_int), false); // read value directly from node with another key with value string
    EXPECT_EQ(test_unsigned_int, 2u); // check if value is not overwritten
    EXPECT_EQ(Parser::read<unsigned int>(node["u2"], test_unsigned_int), false); // read value from an invalid/empty node
    EXPECT_EQ(test_unsigned_int, 2u); // check if value is not overwritten
    EXPECT_EQ(Parser::read<unsigned int>(node["u"], test_unsigned_int), true); // read value directly from node with correct key
    EXPECT_EQ(test_unsigned_int, 5u); // check if value is read correctly
    EXPECT_EQ(Parser::has<unsigned int>(node, "u"), true);
    EXPECT_EQ(Parser::has<unsigned int>(node, "u2"), false);
    EXPECT_EQ(Parser::is<unsigned int>(node["u"]), true);
    EXPECT_EQ(Parser::is<unsigned int>(node["s"]), false);
    EXPECT_EQ(Parser::get<unsigned int>(node, "u", 0u), 5u);
    EXPECT_EQ(Parser::get<unsigned int>(node, "u2", 0u), 0u);
    EXPECT_EQ(Parser::get<unsigned int>(node["u"], 0u), 5u);
    EXPECT_EQ(Parser::get<unsigned int>(node["s"], 0u), 0u);

    bool test_bool = false;
    EXPECT_EQ(Parser::read<bool>(node, "b2", test_bool), false); // read from map but with incorrect key
    EXPECT_EQ(test_bool, false); // check if value is not overwritten
    EXPECT_EQ(Parser::read<bool>(node, "b", test_bool), true); // read from map with correct key
    EXPECT_EQ(test_bool, true); // check if value is read correctly
    test_bool = false;
    EXPECT_EQ(Parser::read<bool>(node["s"], test_bool), false); // read value directly from node with another key with value string
    EXPECT_EQ(test_bool, false); // check if value is not overwritten
    EXPECT_EQ(Parser::read<bool>(node["b2"], test_bool), false); // read value from an invalid/empty node
    EXPECT_EQ(test_bool, false); // check if value is not overwritten
    EXPECT_EQ(Parser::read<bool>(node["b"], test_bool), true); // read value directly from node with correct key
    EXPECT_EQ(test_bool, true); // check if value is read correctly
    EXPECT_EQ(Parser::has<bool>(node, "b"), true);
    EXPECT_EQ(Parser::has<bool>(node, "b2"), false);
    EXPECT_EQ(Parser::is<bool>(node["b"]), true);
    EXPECT_EQ(Parser::is<bool>(node["s"]), false);
    EXPECT_EQ(Parser::get<bool>(node, "b", false), true);
    EXPECT_EQ(Parser::get<bool>(node, "b2", false), false);
    EXPECT_EQ(Parser::get<bool>(node["b"], false), true);
    EXPECT_EQ(Parser::get<bool>(node["s"], false), false);

    std::string test_string = "xyz";
    EXPECT_EQ(Parser::read<std::string>(node, "s2", test_string), false); // read from map but with incorrect key
    EXPECT_EQ(test_string, "xyz"); // check if value is not overwritten
    EXPECT_EQ(Parser::read<std::string>(node, "s", test_string), true); // read from map with correct key
    EXPECT_EQ(test_string, "abc"); // check if value is read correctly
    test_string = "xyz";
    EXPECT_EQ(Parser::read<std::string>(node["s2"], test_string), false); // read value from an invalid/empty node
    EXPECT_EQ(test_string, "xyz"); // check if value is not overwritten
    EXPECT_EQ(Parser::read<std::string>(node["s"], test_string), true); // read value directly from node with correct key
    EXPECT_EQ(test_string, "abc"); // check if value is read correctly
    EXPECT_EQ(Parser::has<std::string>(node, "s"), true);
    EXPECT_EQ(Parser::has<std::string>(node, "s2"), false);
    EXPECT_EQ(Parser::is<std::string>(node["s"]), true);
    EXPECT_EQ(Parser::get<std::string>(node, "s", "xyz"), "abc");
    EXPECT_EQ(Parser::get<std::string>(node, "s2", "xyz"), "xyz");
    EXPECT_EQ(Parser::get<std::string>(node["s"], "xyz"), "abc");
}

TEST(Parser2Test, geometry_common_datatypes)
{
    YAML::Node node;
    node["i"] = 5;

    YAML::Node point2d_yaml;
    point2d_yaml["x"] = 5.0f;
    point2d_yaml["y"] = 6.0f;
    node["point2d"] = point2d_yaml;

    YAML::Node incomplete_point2d_yaml;
    incomplete_point2d_yaml["x"] = 5.0f;
    YAML::Node wrong_point2d_yaml;
    wrong_point2d_yaml["x"] = 5.0f;
    wrong_point2d_yaml["y"] = "abc";

    YAML::Node point3d_yaml;
    point3d_yaml["x"] = 5.0f;
    point3d_yaml["y"] = 6.0f;
    point3d_yaml["z"] = 7.0f;
    node["point3d"] = point3d_yaml;

    YAML::Node xytheta_yaml;
    xytheta_yaml["x"] = 5.0f;
    xytheta_yaml["y"] = 6.0f;
    xytheta_yaml["theta"] = 7.0f;
    node["xytheta"] = xytheta_yaml;

    YAML::Node pose2d_yaml;
    pose2d_yaml["x"] = 5.0f;
    pose2d_yaml["y"] = 6.0f;
    pose2d_yaml["theta"] = 3.0f;
    node["pose2d"] = pose2d_yaml;

    YAML::Node pose2d_large_theta_yaml;
    pose2d_large_theta_yaml["x"] = 5.0f;
    pose2d_large_theta_yaml["y"] = 6.0f;
    pose2d_large_theta_yaml["theta"] = 30.0f;


    Point2D true_point_2d(5.0f, 6.0f);
    Point2D default_point_2d(2.0f, 3.0f);
    Point2D test_point_2d = default_point_2d;
    EXPECT_EQ(Parser::read<Point2D>(node, "point2d2", test_point_2d), false); // read from map but with incorrect key
    EXPECT_EQ(test_point_2d, default_point_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read<Point2D>(node, "point2d", test_point_2d), true); // read from map with correct key
    EXPECT_EQ(test_point_2d, true_point_2d); // check if value is read correctly
    test_point_2d = default_point_2d;
    EXPECT_EQ(Parser::read(node["i"], test_point_2d), false); // read value directly from node with another key with value int
    EXPECT_EQ(test_point_2d, default_point_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["point2d2"], test_point_2d), false); // read value from an invalid/empty node
    EXPECT_EQ(test_point_2d, default_point_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(incomplete_point2d_yaml, test_point_2d), false); // read value directly from node with missing keys
    EXPECT_EQ(test_point_2d, default_point_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(wrong_point2d_yaml, test_point_2d), false); // read value directly from node with key with value string
    EXPECT_EQ(test_point_2d, default_point_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["point2d"], test_point_2d), true); // read value directly from node with correct key
    EXPECT_EQ(test_point_2d, true_point_2d); // check if value is read correctly
    EXPECT_EQ(Parser::has<Point2D>(node, "point2d"), true);
    EXPECT_EQ(Parser::has<Point2D>(node, "point2d2"), false);
    EXPECT_EQ(Parser::is<Point2D>(node["point2d"]), true);
    EXPECT_EQ(Parser::is<Point2D>(node["i"]), false);
    EXPECT_EQ(Parser::get<Point2D>(node, "point2d", default_point_2d), true_point_2d);
    EXPECT_EQ(Parser::get<Point2D>(node, "point2d2", default_point_2d), default_point_2d);
    EXPECT_EQ(Parser::get<Point2D>(node["point2d"], default_point_2d), true_point_2d);
    EXPECT_EQ(Parser::get<Point2D>(node["i"], default_point_2d), default_point_2d);

    Point3D true_point_3d(5.0f, 6.0f, 7.0f);
    Point3D default_point_3d(2.0f, 3.0f, 4.0f);
    Point3D test_point_3d = default_point_3d;
    EXPECT_EQ(Parser::read<Point3D>(node, "point3d2", test_point_3d), false); // read from map but with incorrect key
    EXPECT_EQ(test_point_3d, default_point_3d); // check if value is not overwritten
    EXPECT_EQ(Parser::read<Point3D>(node, "point3d", test_point_3d), true); // read from map with correct key
    EXPECT_EQ(test_point_3d, true_point_3d); // check if value is read correctly
    test_point_3d = default_point_3d;
    EXPECT_EQ(Parser::read(node["i"], test_point_3d), false); // read value directly from node with another key with value int
    EXPECT_EQ(test_point_3d, default_point_3d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["point3d2"], test_point_3d), false); // read value from an invalid/empty node
    EXPECT_EQ(test_point_3d, default_point_3d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["point3d"], test_point_3d), true); // read value directly from node with correct key
    EXPECT_EQ(test_point_3d, true_point_3d); // check if value is read correctly
    EXPECT_EQ(Parser::has<Point3D>(node, "point3d"), true);
    EXPECT_EQ(Parser::has<Point3D>(node, "point3d2"), false);
    EXPECT_EQ(Parser::is<Point3D>(node["point3d"]), true);
    EXPECT_EQ(Parser::is<Point3D>(node["i"]), false);
    EXPECT_EQ(Parser::get<Point3D>(node, "point3d", default_point_3d), true_point_3d);
    EXPECT_EQ(Parser::get<Point3D>(node, "point3d2", default_point_3d), default_point_3d);
    EXPECT_EQ(Parser::get<Point3D>(node["point3d"], default_point_3d), true_point_3d);
    EXPECT_EQ(Parser::get<Point3D>(node["i"], default_point_3d), default_point_3d);

    XYTheta true_xytheta(5.0f, 6.0f, 7.0f);
    XYTheta default_xytheta(2.0f, 3.0f, 4.0f);
    XYTheta test_xytheta = default_xytheta;
    EXPECT_EQ(Parser::read<XYTheta>(node, "xytheta2", test_xytheta), false); // read from map but with incorrect key
    EXPECT_EQ(test_xytheta, default_xytheta); // check if value is not overwritten
    EXPECT_EQ(Parser::read<XYTheta>(node, "xytheta", test_xytheta), true); // read from map with correct key
    EXPECT_EQ(test_xytheta, true_xytheta); // check if value is read correctly
    test_xytheta = default_xytheta;
    EXPECT_EQ(Parser::read(node["i"], test_xytheta), false); // read value directly from node with another key with value int
    EXPECT_EQ(test_xytheta, default_xytheta); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["xytheta2"], test_xytheta), false); // read value from an invalid/empty node
    EXPECT_EQ(test_xytheta, default_xytheta); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["xytheta"], test_xytheta), true); // read value directly from node with correct key
    EXPECT_EQ(test_xytheta, true_xytheta); // check if value is read correctly
    EXPECT_EQ(Parser::has<XYTheta>(node, "xytheta"), true);
    EXPECT_EQ(Parser::has<XYTheta>(node, "xytheta2"), false);
    EXPECT_EQ(Parser::is<XYTheta>(node["xytheta"]), true);
    EXPECT_EQ(Parser::is<XYTheta>(node["i"]), false);
    EXPECT_EQ(Parser::get<XYTheta>(node, "xytheta", default_xytheta), true_xytheta);
    EXPECT_EQ(Parser::get<XYTheta>(node, "xytheta2", default_xytheta), default_xytheta);
    EXPECT_EQ(Parser::get<XYTheta>(node["xytheta"], default_xytheta), true_xytheta);
    EXPECT_EQ(Parser::get<XYTheta>(node["i"], default_xytheta), default_xytheta);

    Pose2D true_pose_2d(5.0f, 6.0f, 3.0f);
    Pose2D default_pose_2d(2.0f, 3.0f, 1.0f);
    Pose2D test_pose_2d = default_pose_2d;
    EXPECT_EQ(Parser::read<Pose2D>(node, "pose2d2", test_pose_2d), false); // read from map but with incorrect key
    EXPECT_EQ(test_pose_2d, default_pose_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read<Pose2D>(node, "pose2d", test_pose_2d), true); // read from map with correct key
    EXPECT_EQ(test_pose_2d, true_pose_2d); // check if value is read correctly
    test_pose_2d = default_pose_2d;
    EXPECT_EQ(Parser::read(node["i"], test_pose_2d), false); // read value directly from node with another key with value int
    EXPECT_EQ(test_pose_2d, default_pose_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["pose2d2"], test_pose_2d), false); // read value from an invalid/empty node
    EXPECT_EQ(test_pose_2d, default_pose_2d); // check if value is not overwritten
    EXPECT_EQ(Parser::read(node["pose2d"], test_pose_2d), true); // read value directly from node with correct key
    EXPECT_EQ(test_pose_2d, true_pose_2d); // check if value is read correctly
    EXPECT_EQ(Parser::has<Pose2D>(node, "pose2d"), true);
    EXPECT_EQ(Parser::has<Pose2D>(node, "pose2d2"), false);
    EXPECT_EQ(Parser::is<Pose2D>(node["pose2d"]), true);
    EXPECT_EQ(Parser::is<Pose2D>(node["i"]), false);
    EXPECT_EQ(Parser::get<Pose2D>(node, "pose2d", default_pose_2d), true_pose_2d);
    EXPECT_EQ(Parser::get<Pose2D>(node, "pose2d2", default_pose_2d), default_pose_2d);
    EXPECT_EQ(Parser::get<Pose2D>(node["pose2d"], default_pose_2d), true_pose_2d);
    EXPECT_EQ(Parser::get<Pose2D>(node["i"], default_pose_2d), default_pose_2d);
    EXPECT_EQ(Parser::get<Pose2D>(pose2d_large_theta_yaml, default_pose_2d), Pose2D(5.0f, 6.0f, 0.0f));

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
