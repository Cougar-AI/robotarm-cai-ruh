# find_ranges.py

import mujoco
import mujoco.viewer
import time

XML_PATH = "RobotArm-Ethan.xml"

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    # We use the 'launch' method (not launch_passive) to get the full interactive GUI
    # This blocks the script, giving you full control via the mouse/menus
    mujoco.viewer.launch(model, data)

if __name__ == "__main__":
    main()