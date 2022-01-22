cmake -DOpen3D_ROOT="D:/carlos/Program Files/open3d141_r/" -S . -B ./build
cmake --build ./build --config Release
.\build\Release\RegistrationRANSAC.exe ..\..\data\pointcloud\bun045.ply ..\..\data\pointcloud\bun000.ply 0.005 --visualize --fix_seed