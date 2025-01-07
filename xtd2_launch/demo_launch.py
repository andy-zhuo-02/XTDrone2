from xtd2_launch_utils.utils import *

def run(cmd, cwd='.'):
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd=cwd)
    return process_handle


def demo_launch():
    world = "aruco"
    
    h1 = px4_launch("gz_x500", world=world)
    h2 = gazebo_launch(world=world)

    try:
        while h1.poll() is None or h2.poll() is None:
            h1.wait()
            h2.wait()
    except KeyboardInterrupt:
        h1.terminate()
        h2.terminate()
        exit(0)


if __name__ == "__main__":
    demo_launch()
