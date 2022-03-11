# YAML common

A ROS package with helper functions to parse and manipulate YAML data

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
