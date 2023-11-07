import os
import numpy as np
import struct
import open3d
import demo.pcd_demo as demo

def read_bin_velodyne(path):
    pc_list=[]
    with open(path,'rb') as f:
        content=f.read()
        pc_iter=struct.iter_unpack('ffff',content)
        for idx,point in enumerate(pc_iter):
            pc_list.append([point[0],point[1],point[2]])
    return np.asarray(pc_list,dtype=np.float32)

def main():
    path='demo/data/kitti/000008.bin'
    # path = 'tests/data/kitti/training/velodyne/000000.bin'
    pc_np =read_bin_velodyne(path)

    results = demo.main(path)
    bboxes_3d = results['predictions'][0]['bboxes_3d']
    
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pc_np)
    rot_axis = 2
    
    num_pts = np.zeros(len(bboxes_3d))
    for i in range(len(bboxes_3d)):
        center = bboxes_3d[i][0:3]
        dim = bboxes_3d[i][3:6]
        yaw = np.zeros(3)
        yaw[rot_axis] = bboxes_3d[i][6]
        rot_mat = open3d.geometry.get_rotation_matrix_from_xyz(yaw)

        # bottom center to gravity center
        center[rot_axis] += dim[rot_axis] / 2
   
        box3d = open3d.geometry.OrientedBoundingBox(center, rot_mat, dim)

        indices = box3d.get_point_indices_within_bounding_box(
                    pcd.points)
        num_pts[i] = len(indices)
        
    print(num_pts)
    # example=read_bin_velodyne(path)
    # # From numpy to Open3D
    # pcd.points= open3d.open3d.utility.Vector3dVector(example)
    # open3d.open3d.visualization.draw_geometries([pcd])

if __name__=="__main__":
    main()