work on open3d 0.14.1
# compile
## windows
```bash
cmake -DOpen3D_ROOT="D:/carlos/Program Files/open3d141_r/" -S . -B ./build
cmake --build ./build --config Release
```

## ubuntu
```bash
cmake -DCMAKE_BUILD_TYPE=Relese -DOpen3D_ROOT="your open3d install path" -S . -B ./build
make -j8
```

# test
## windows
RegistrationRANSAC.exe source target voxel_size --visualize --fix_seed \
add --visualize to show pointcloud \
add --fix_seed to fix the random seed to a given seed "123456" \
e.g

```bash
.\build\Release\RegistrationRANSAC.exe ..\..\data\pointcloud\bun045.ply ..\..\data\pointcloud\bun000.ply 0.005 --visualize --fix_seed
```

