import sys
import open3d as o3d
import os

if __name__ == "__main__":
    # for i in sys.argv:
    #     print(i)
    pcd_filename = sys.argv[1]
    down_voxel_size = sys.argv[2]

    print("pcd: " + pcd_filename)
    print("voxel size: " + str(down_voxel_size))

    pcd = o3d.io.read_point_cloud(filename=pcd_filename)
    if len(sys.argv) >= 4:
        if sys.argv[3] == 'visualize':
            o3d.visualization.draw_geometries([pcd],
                                              window_name="pcd",
                                              width=960, height=900, left=960, top=100)

    pcd_down = pcd.voxel_down_sample(float(down_voxel_size))
    (filename, extension) = os.path.splitext(pcd_filename)
    filename_save = filename + "down_{}.pcd".format(down_voxel_size)
    o3d.io.write_point_cloud(filename=filename_save, pointcloud=pcd_down)
    print("file saved to " + filename_save)
