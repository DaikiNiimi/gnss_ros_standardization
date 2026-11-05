# Using the installed package

## C++

```cmake
find_package(gnss_ros_standardization REQUIRED)
add_executable(my_subscriber my_subscriber.cpp)
target_link_libraries(my_subscriber gnss_ros_standardization::gnss_utils)
```

`gnss_ros_standardization::gnss_utils` brings the message types, the helper library
and the RTKLIB compile definitions (constellations, `NEXOBS`). For direct RTKLIB
access link `gnss_ros_standardization::rtkcore_lib` and include
`gnss_ros_standardization/rtklib_compat.hpp`. Example:
[test/install_consumer](../test/install_consumer/).

## Python

```python
from gnss_ros_standardization.msg import GnssObservations, GnssSolution
```

## Check an installation

```bash
source install/setup.bash
bash test/check_install.sh "$PWD/install/gnss_ros_standardization"
```

Builds an external package against the installed targets and exchanges all six
message types between C++ and Python.

Publishers and subscribers must use the same installation of the message types.
