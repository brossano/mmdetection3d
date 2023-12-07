import os
import numpy as np
import struct
import open3d
import demo.pcd_demo as demo
import pdb
import pdb

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
    pcd=open3d.open3d.geometry.PointCloud()
    # breakpoint()
    # path='demo/data/kitti/000008.bin'
    path = 'tests/data/kitti/training/velodyne/000000.bin'
    pc_np =read_bin_velodyne(path)

    example=read_bin_velodyne(path)
    # From numpy to Open3D
    pcd.points= open3d.open3d.utility.Vector3dVector(example)
    open3d.open3d.visualization.draw_geometries([pcd])

if __name__=="__main__":
    main()