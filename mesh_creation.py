import open3d as o3d
import numpy as np

if __name__ == "__main__":
    print("Testing IO for point cloud ...")
    pcd = o3d.io.read_point_cloud("./cloud_data/pcl_out_time40-163000000.pcd")
    print(pcd)
    o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)

    print(np.asarray(pcd.points))
    '''If you want to visualize the point cloud, open a new terminal and do:"
    $ export XDG_SESSION_TYPE=x11
    $ export GDK_BACKEND=x11
    before calling the function below
    '''
    #o3d.visualization.draw_geometries([pcd])

    print("Testing IO for meshes ...")
    mesh = o3d.io.read_triangle_mesh("./pcl_data/pcl_out_time120-193000000.ply")
    #mesh = o3d.io.read_triangle_mesh("./cloud_data/pcl_out_time16-217000000.ply")
    print(mesh)
    o3d.io.write_triangle_mesh("copy_of_knot.ply", mesh)

    #pcd = o3d.io.read_point_cloud("./pcl_data/pcl_out_time120-193000000.ply")
    pcd = o3d.io.read_point_cloud("./cloud_data/pcl_out_time16-217000000.ply")
    pcd.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)
    o3d.io.write_triangle_mesh("mesh.ply", mesh)

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("mesh_12m.ply")
    #pcd = o3d.io.read_point_cloud("mesh.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])