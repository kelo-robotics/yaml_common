# YAML common

[![build status](https://git.locomotec.com:444/kelo/common/yaml_common/badges/master/build.svg)](https://git.locomotec.com:444/kelo/common/yaml_common/commits/master)

A ROS package with helper functions to parse and manipulate YAML data

## Dependencies

- yaml-cpp (`sudo apt install libyaml-cpp-dev`)
- [geometry_common](https://github.com/kelo-robotics/geometry_common) (optional)

To build without `geometry_common`, execute
```bash
catkin build yaml_common -DBUILD_WITH_GEOMETRY_COMMON=OFF
```

## Documentation

We use [Doxygen](https://www.doxygen.nl/index.html) for code documentation.

**Note**: Building the documentation is disabled by default.

- To build the code documentation, Doxygen needs to be installed. On debian based
  systems, this can be achieved with
  ```bash
  sudo apt install doxygen
  ```

- Documentation can be built using the flag `-DBUILD_DOC=ON`
  ```bash
  catkin build yaml_common -DBUILD_DOC=ON
  ```

- The documentation will be generated at
  `<YOUR_CATKIN_WS>/build/yaml_common/docs/html/index.html`

## Test

Run unit tests with

```bash
catkin test --this
```

**Note**: Requires `GTest` package (`sudo apt install libgtest-dev`)
