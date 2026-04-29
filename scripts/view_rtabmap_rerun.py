#!/usr/bin/env python3                                                                                                                                 
import argparse                                       
from pathlib import Path
import numpy as np
import rerun as rr
from plyfile import PlyData                                                                                                                            

def load_ply(path):                                                                                                                                    
    ply = PlyData.read(str(path))                     
    v = ply["vertex"].data                                                                                                                             
    xyz = np.stack([v["x"], v["y"], v["z"]], axis=1)
    rgb = None                                                                                                                                         
    if {"red", "green", "blue"}.issubset(v.dtype.names):
        rgb = np.stack([v["red"], v["green"], v["blue"]], axis=1).astype(np.uint8)                                                                     
    return xyz, rgb                                   
                                                                                                                                                        
def load_tum_poses(path):                                                                                                                              
    poses = []
    for line in Path(path).read_text().splitlines():                                                                                                   
        line = line.strip()                           
        if not line or line.startswith("#"):
            continue                                                                                                                                   
        ts, tx, ty, tz, qx, qy, qz, qw = map(float, line.split())
        poses.append((ts, np.array([tx, ty, tz]), np.array([qx, qy, qz, qw])))                                                                         
    return poses                                                                                                                                       

def main():                                                                                                                                            
    ap = argparse.ArgumentParser()                    
    ap.add_argument("--dir", default="rtabmap_maps")
    args = ap.parse_args()                                                                                                                             
    d = Path(args.dir)
                                                                                                                                                        
    rr.init("rtabmap", spawn=True)                    
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)                                                                                       
                                                    
    xyz, rgb = load_ply(d / "rtabmap_cloud.ply")                                                                                                               
    rr.log("/map/cloud", rr.Points3D(xyz, colors=rgb, radii=0.01), static=True)
                                                                                                                                                        
    poses_file = next(d.glob("poses*.txt"), None)     
    if poses_file:                                                                                                                                     
        poses = load_tum_poses(poses_file)            
        traj = np.array([t for _, t, _ in poses])                                                                                                      
        rr.log("/map/trajectory",
                rr.LineStrips3D([traj], colors=[255, 200, 0]), static=True)                                                                             
        for i, (ts, t, q) in enumerate(poses):        
            rr.set_time_seconds("stamp", ts)                                                                                                           
            rr.set_time_sequence("frame", i)                                                                                                           
            rr.log("/map/camera", rr.Transform3D(translation=t, quaternion=q))
                                                                                                                                                        
if __name__ == "__main__":                            
    main()