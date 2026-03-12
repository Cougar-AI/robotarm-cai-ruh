# bake_workspace.py

import time
import mujoco
import numpy as np
import random
import open3d as o3d

# --- Configuration ---
XML_PATH = "RobotArm-Ethan.xml"
SITE_NAME = "end_effector"
N_SAMPLES = 1_000_000      
DEXTEROUS_THRESHOLD = 3.0 
ALPHA_VALUE = 0.05       
# ---------------------

def create_mesh_from_points(points, filename):
    if len(points) < 100:
        print(f"Skipping {filename}: Not enough points ({len(points)}).")
        return

    print(f"Reconstructing Surface for {filename} ({len(points)} pts)...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    try:
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, ALPHA_VALUE)
        mesh.compute_vertex_normals()
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=3)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(filename, mesh)
        print(f"Saved {filename}")
    except Exception as e:
        print(f"Meshing failed for {filename}: {e}")

def main():
    print(f"Loading model from {XML_PATH}...")
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    
    if site_id == -1:
        print(f"ERROR: Site '{SITE_NAME}' not found in XML!")
        return

    print(f"Starting Sampling ({N_SAMPLES} points)...")
    start_time = time.time()
    
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    
    reachable = []
    dexterous = []
    
    min_cond = 9999.0
    valid_count = 0
    
    for i in range(N_SAMPLES):
        # Random sampling
        q = np.zeros(model.nq)
        for j in range(model.nq):
            lim = model.jnt_range[j]
            if lim[0] < lim[1]:
                q[j] = random.uniform(lim[0], lim[1])
            else:
                q[j] = random.uniform(-3.14, 3.14)
        
        data.qpos = q
        
        # --- THE FIX: Use mj_forward instead of mj_kinematics ---
        mujoco.mj_forward(model, data)
        
        pos = data.site_xpos[site_id].copy()
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        
        # SVD
        s = np.linalg.svd(jacp, compute_uv=False)
        
        # DEBUG PRINT FOR FIRST SAMPLE
        if i == 0:
            print("\n--- DEBUG SAMPLE 0 ---")
            print(f"QPos: {np.round(q, 2)}")
            print(f"Position: {np.round(pos, 2)}")
            print(f"Singular Values: {s}")
            print("----------------------\n")

        if s[-1] > 1e-6:
            cond = s[0] / s[-1]
            valid_count += 1
            if cond < min_cond: min_cond = cond
        else:
            cond = 9999.0
            
        reachable.append(pos)
        
        if cond < DEXTEROUS_THRESHOLD:
            dexterous.append(pos)
            
    duration = time.time() - start_time
    print(f"Done in {duration:.2f}s.")
    print(f"Valid Samples: {valid_count}/{N_SAMPLES}")
    print(f"Best Condition Number: {min_cond:.2f}")
    
    create_mesh_from_points(np.array(reachable), "workspace_reachable.stl")
    create_mesh_from_points(np.array(dexterous), "workspace_dexterous.stl")

if __name__ == "__main__":
    main()