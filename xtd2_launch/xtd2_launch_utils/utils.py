import subprocess

def run(cmd, cwd='.'):
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd=cwd)
    return process_handle

def gazebo_launch(model_store="~/.simulation-gazebo", 
                  world="default"):
    print("> Launching gazebo simulation...")

    gazebo_cmd = f'GZ_SIM_RESOURCE_PATH={model_store}/models gz sim -r {model_store}/worlds/{world}.sdf'

    print(gazebo_cmd)

    world_launch_handle = run(gazebo_cmd)

    return world_launch_handle

def px4_launch(model_name, 
               pose=(0,0,0,0,0,0),
               world="default"):
    print("> Launching PX4...")

    sys_autostart = 4001  # Mapping from model_name

    x, y, z, r, p, y = pose

    px4_cmd = f"PX4_GZ_WORLD={world} PX4_SYS_AUTOSTART={sys_autostart} PX4_SIM_MODEL={model_name} PX4_GZ_MODEL_POSE='{x},{y},{z},{r},{p},{y}' PX4_GZ_STANDALONE=1 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4"

    print(px4_cmd)

    px4_launch_handle = run(px4_cmd)

    return px4_launch_handle
