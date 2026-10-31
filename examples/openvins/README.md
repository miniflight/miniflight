This builds the optional OpenVINS backend for `miniflight.openvins.OpenVINS`.
It accepts paired IMU samples and camera frames. Estimates use a local z-down
frame whose origin and heading can be registered once to an external frame.

Install Eigen, OpenCV, Boost and Ceres, then build against the upstream source.
The verified upstream revision is `69488123ed9362dd44b6f28e7f4680abbff1442b`.

```sh
git clone https://github.com/rpng/open_vins /tmp/open_vins
git -C /tmp/open_vins checkout 69488123ed9362dd44b6f28e7f4680abbff1442b
cmake -S examples/openvins -B target/aigp/.runtime/openvins-build -DOPENVINS_SOURCE=/tmp/open_vins -DCMAKE_BUILD_TYPE=Release
cmake --build target/aigp/.runtime/openvins-build -j2
```

OpenVINS source and its license stay in that checkout. No ROS installation is used.
Camera and IMU calibration must describe the connected sensors. The AIGP example
is in `target/aigp/config/vio/`; its velocity experiment uses the existing repeat
runner with `--velocity-estimator target/aigp/config/vio/velocity.json --camera`.
Camera packets are required to replay this estimator.
