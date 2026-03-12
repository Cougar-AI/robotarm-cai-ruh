# trace_circle.py

import time
import mujoco
import mujoco.viewer
import numpy as np

# --- Configuration ---
# XML_PATH = "RobotArm-Ethan.xml"
XML_PATH = "RArmEthanWorkspace.xml"
SITE_NAME = "end_effector"
CIRCLE_RADIUS = 0.15       
# MOVED TARGET: Further out (0.55) and higher (0.45) to give the arm room
CIRCLE_CENTER = np.array([0.55, 0.0, 0.45]) 
SPEED = 0.8                
DAMPING = 1e-2             
MAX_VELOCITY = 2.0        
# ---------------------

# --- Global State Flags ---
# We store the state here so it doesn't get overwritten by the viewer sync
SHOW_REACHABLE = True  
SHOW_DEXTEROUS = True

def circle_target(t):
    """Returns a generic point on a circle in the YZ plane."""
    y = CIRCLE_RADIUS * np.cos(SPEED * t)
    z = CIRCLE_RADIUS * np.sin(SPEED * t)
    return CIRCLE_CENTER + np.array([0.0, y, z])

def key_callback(keycode):
    # MuJoCo keycodes: '1' = 49, '2' = 50
    global SHOW_REACHABLE, SHOW_DEXTEROUS
    
    # Actually, the easiest way in the Passive Viewer is to toggle Groups.
    # In your XML, assign group="1" to Reachable and group="2" to Dexterous.
    if keycode == 49: # 1
        SHOW_REACHABLE = not SHOW_REACHABLE
        print(f"Reachable Visible: {SHOW_REACHABLE}")
    if keycode == 50: # 2
        SHOW_DEXTEROUS = not SHOW_DEXTEROUS
        print(f"Dexterous Visible: {SHOW_DEXTEROUS}")

def main():
    print(f"Loading model from {XML_PATH}...")
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        print("Viewer launched. Mode: KINEMATIC ONLY (No Physics Explosions)")
        
        print("\n--- CONTROLS ---")
        print("Press '1' to toggle REACHABLE (Grey)")
        print("Press '2' to toggle DEXTEROUS (Cyan)")
        print("----------------\n")
        
        # Ensure groups are on by default
        viewer.opt.geomgroup[1] = 1
        viewer.opt.geomgroup[2] = 1
        
        # Initial Pose
        q0 = np.array([0, -0.5, 1.5, 0, 0.5, 0])
        data.qpos = q0.copy()
        mujoco.mj_forward(model, data)

        start_time = time.time()
        
        # List to store the path history
        path_history = []

        while viewer.is_running():
            t = time.time() - start_time
            target_pos = circle_target(t)
            
            # --- ENFORCE VISIBILITY STATE ---
            # We apply the Python flags to the Viewer Options every single frame
            # This prevents the viewer from resetting them
            viewer.opt.geomgroup[1] = 1 if SHOW_REACHABLE else 0
            viewer.opt.geomgroup[2] = 1 if SHOW_DEXTEROUS else 0

            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
            current_pos = data.site_xpos[site_id]
            error = target_pos - current_pos
            
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

            # Store current position every 5th frame to save memory/FPS
            if len(path_history) == 0 or np.linalg.norm(current_pos - path_history[-1]) > 0.01:
                path_history.append(current_pos.copy())
                # Limit tail length to 200 points
                if len(path_history) > 200:
                    path_history.pop(0)
                
            # --- Inverse Kinematics ---
            jac_t = jacp.T
            lambda_sq = DAMPING ** 2
            hessian = jacp @ jac_t + lambda_sq * np.eye(3)
            
            try:
                j_pinv = jac_t @ np.linalg.inv(hessian)
                dq_primary = j_pinv @ error
            except np.linalg.LinAlgError:
                dq_primary = np.zeros(model.nv)

            # Null Space (Secondary Task: Stay near q0)
            error_joint = (q0 - data.qpos) * 0.05
            I = np.eye(model.nv)
            null_space_projector = I - j_pinv @ jacp
            dq_secondary = null_space_projector @ error_joint

            # Combine
            dq = dq_primary + dq_secondary
            dq = np.clip(dq, -MAX_VELOCITY, MAX_VELOCITY)

            # Integrate Position
            data.qpos += dq * 0.1
            
            # --- SAFETY CLAMPS ---
            # This is now safe because we are NOT running physics dynamics
            for i in range(model.njnt):
                min_range = model.jnt_range[i][0]
                max_range = model.jnt_range[i][1]
                if min_range < max_range: 
                    data.qpos[i] = np.clip(data.qpos[i], min_range, max_range)

            # --- THE FIX ---
            # Use mj_forward (Kinematics) instead of mj_step (Physics)
            # This updates the visual model to match data.qpos without calculating forces.
            mujoco.mj_forward(model, data) 

            # Visualization
            viewer.user_scn.ngeom = 1
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[0],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.015, 0, 0],
                pos=target_pos,
                mat=np.eye(3).flatten(),
                rgba=[0, 1, 0, 1] 
            )
            
            # Draw the path trail
            for pos in path_history:
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.005, 0, 0], # Tiny dots
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=[0, 1, 1, 0.5] # Cyan, semi-transparent
                )
                viewer.user_scn.ngeom += 1
                
            viewer.sync()
            time.sleep(0.01)

if __name__ == "__main__":
    main()