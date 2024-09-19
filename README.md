# Surface Edge Explorer (SEE++)

## Description

SEE++ is a density-based Next Best View (NBV) planning approach for obtaining 3D scene observations.

**This repository provides a Python interface for the [SEE](https://github.com/robotic-esp/see-public) library. Additionally, it includes a Windows-compatible version.**

## Dependencies

- [PCL 1.14.1](https://github.com/PointCloudLibrary/pcl/releases)
  - You need to fix the compatibility problem of FLANN with C++ 14.
  - [**ACTION**] Name `PCL` in your environment.
- [NLOPT](https://github.com/stevengj/nlopt)
  - Install [NLOPT](https://github.com/stevengj/nlopt) by the given link and compile locally.
  - [**ACTION**] Name `NLOPT` in your environment.
- [Lemon](https://github.com/seqan/lemon)
  - Header-only, located in the `./3rdParty/` directory.
- [nlohmann Json](https://github.com/nlohmann/json)
  - Header-only, located in the `./3rdParty/` directory.
- [pybind11](https://github.com/pybind/pybind11)
  - Header-only, located in the `./3rdParty/` directory.

## Installation

*Built using C++ 14.*

### Python (or via PIP)

``` shell
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

// Via pip: TODO

You can import the `pysee.pyd` file located in `build/Release` using `import pysee`. If you encounter any issues related to `dll` files, please refer to the `./all-in-one/*.dll` directory.

### Compiling on Windows

- Visual Studio 2022 x64

## Usage

### C++ Example

``` C++
#include "SEE.h"

int main(int argc, char* argv[])
{
  SEE see("./config.json");

  SeePointCloudPtr cloud(new pcl::PointCloud<SeePoint>);

  if (pcl::io::loadPCDFile<SeePoint>("test.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file your_point_cloud.pcd\n");
    return -1;
  }

  SeeView view, nbv;

  // Set the current view (camera position and orientation vector of the camera)
  view.x = -1.25516047;
  view.y = 0.16405614;
  view.z = 0.90033961;
  view.view_x = 0.81845733;
  view.view_y = -0.11847473;
  view.view_z = -0.56222001;

  // the next best view will be set as nbv
  see.SearchNBVOnce(cloud, view, nbv);

  print_view(nbv);

  return 0;
}
```

For additional examples, refer to `./test_main.cpp`

### Python Example

```Python
import pysee
import numpy as np

# define the camera and algorithm parameters
see = pysee.init("../config.json")

# load the point cloud
points = np.loadtxt("../test.txt", dtype=np.float32)[:, 0:3].tolist()

# set the current view, position and orientation(pointing direction)
current_v = np.array([-1.25516047, 0.16405614, 0.90033961, 0.81845733, -0.11847473, -0.56222001], dtype=np.float32).tolist()

# search for the next view
nbv = see.search_nbv_once(points, current_v)

print(f"Next view: {nbv}")
```

## Project Original Owners

- Rowan Border <rborder@robots.ox.ac.uk>

Modified by:

- Leihui Li <leihui@mpe.au.dk>

## References

<https://github.com/robotic-esp/see-public>

## Licence

MIT Licence

## Copyright

Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
