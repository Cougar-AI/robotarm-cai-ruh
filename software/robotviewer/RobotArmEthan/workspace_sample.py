# workspace_sample.py

import time
import mujoco
import mujoco.viewer
import numpy as np
import random

# --- Configuration ---
XML_PATH = "RobotArm-Ethan.xml"
SITE_NAME = "end_effector"

# Sampling Config
N_SAMPLES = 100000        # 100k samples gives a very dense map
VOXEL_SIZE = 0.03         # 3cm cubes. Smaller = smoother but slower to render.

# Dexterity Config
# A condition number of 1.0 is perfect. 
# 5.0 is very good. 20.0 is okay. >50 is bad.
DEXTEROUS_THRESHOLD = 15.0 
# ---------------------

# Global voxel map: Key=(x,y,z integer indices), Value=Best Condition Number found there
voxel_map = {}

def generate_reachability_map(model, data, site_name, n_samples):
    """
    Generates a Voxel Map of the robot's capabilities.
    """
    print(f"\n--- STARTING PRE-COMPUTE PHASE ({n_samples} samples) ---")
    print("Calculating kinematics... (This might take a few seconds)")
    
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    
    # Stats tracking
    min_cond = 9999.0
    max_cond = 0.0
    
    start_time = time.time()
    
    # Re-use arrays to save memory
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    
    for i in range(n_samples):
        # 1. Random Configuration
        random_q = np.zeros(model.nq)
        for j in range(model.nq):
            lim = model.jnt_range[j]
            # Respect limits if they exist
            if lim[0] < lim[1]:
                random_q[j] = random.uniform(lim[0], lim[1])
            else:
                # If continuous joint, sample -pi to pi
                random_q[j] = random.uniform(-3.14159, 3.14159)
                
        # 2. Forward Kinematics
        data.qpos = random_q
        mujoco.mj_kinematics(model, data)
        
        # 3. Get Data
        pos = data.site_xpos[site_id]
        
        # 4. Calculate Manipulability (Translational)
        # We check the Jacobian to see how "singular" the robot is at this point
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        
        # Condition Number calculation
        # If singular values are close to 0, cond -> infinity
        try:
            # We only look at Translation (3x6) for now
            # If you want full 6-DOF dexterity, stack jacp and jacr
            s = np.linalg.svd(jacp, compute_uv=False)
            cond_num = s[0] / s[-1] if s[-1] > 1e-6 else 9999.0
        except:
            cond_num = 9999.0

        # Update global stats
        if cond_num < min_cond: min_cond = cond_num
        if cond_num > max_cond and cond_num < 9000: max_cond = cond_num

        # 5. Voxelization (Quantize the position)
        # Convert continuous float position to integer grid coordinates
        vx = int(pos[0] / VOXEL_SIZE)
        vy = int(pos[1] / VOXEL_SIZE)
        vz = int(pos[2] / VOXEL_SIZE)
        
        key = (vx, vy, vz)
        
        # Store the BEST (lowest) condition number found in this voxel
        if key not in voxel_map:
            voxel_map[key] = cond_num
        else:
            if cond_num < voxel_map[key]:
                voxel_map[key] = cond_num
                
        # Progress Bar
        if i % (n_samples // 10) == 0:
            print(f"{i / n_samples * 100:.0f}% complete...")

    duration = time.time() - start_time
    print(f"100% complete in {duration:.2f}s.")
    print("\n--- RESULTS ---")
    print(f"Total Voxels Found: {len(voxel_map)} (approx volume: {len(voxel_map) * VOXEL_SIZE**3:.4f} m^3)")
    print(f"Best Condition Number Found: {min_cond:.4f}")
    print(f"Worst (Non-Singular) Condition Found: {max_cond:.4f}")
    
    if min_cond > DEXTEROUS_THRESHOLD:
        print(f"\n[WARNING] Your threshold ({DEXTEROUS_THRESHOLD}) is lower than the best possible score ({min_cond}).")
        print("--> Nothing will show up as Dexterous! Increase your threshold variable.")

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    # Run the math before opening the window
    generate_reachability_map(model, data, SITE_NAME, N_SAMPLES)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("\n--- VISUALIZER RUNNING ---")
        print(f"VOXEL SIZE: {VOXEL_SIZE}m")
        print(f"GREY = Reachable (Cond > {DEXTEROUS_THRESHOLD})")
        print(f"CYAN = Dexterous (Cond < {DEXTEROUS_THRESHOLD})")
        
        # Set robot to Home Pose
        data.qpos = [0, -0.5, 1.5, 0, 0.5, 0]
        mujoco.mj_forward(model, data)

        while viewer.is_running():
            
            # Render the Voxel Map
            # We iterate over our dictionary and draw boxes
            for coord, cond in voxel_map.items():
                
                # Limit render count to prevent lag if map is huge
                if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom: 
                    break

                # Recover float position from integer coordinate
                pos = np.array(coord) * VOXEL_SIZE + (VOXEL_SIZE / 2)
                
                # Color Logic
                if cond < DEXTEROUS_THRESHOLD:
                    # Cyan (Good Dexterity)
                    rgba = [0, 1, 1, 0.3] 
                else:
                    # Grey (Reachable but strained)
                    rgba = [0.5, 0.5, 0.5, 0.1]
                
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[VOXEL_SIZE/2 - 0.001, VOXEL_SIZE/2 - 0.001, VOXEL_SIZE/2 - 0.001], # Box half-size
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=rgba
                )
                viewer.user_scn.ngeom += 1

            viewer.sync()
            time.sleep(0.03) # Slow down loop to save CPU

if __name__ == "__main__":
    main()