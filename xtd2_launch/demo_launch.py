from xtd2_launch_utils.utils import *

def run(cmd, cwd='.'):
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd=cwd)
    return process_handle


def demo_launch():
    world = "indoor1"
    # world = "default"

    handles = [
        # Launch Gazebo simulation
        gazebo_launch(world=world),

        # Launch PX4 model and sitl
        px4_launch("gz_x500", world=world, pose=(0, 7.5, 0.5, 0, 0, 0)),
    ]
    
    try:
        while True:
            for h in handles:
                if h.poll() is not None:
                    break
            else:
                h.wait()
    except KeyboardInterrupt:
        for h in handles:
            h.terminate()
        exit(0)


if __name__ == "__main__":
    demo_launch()
