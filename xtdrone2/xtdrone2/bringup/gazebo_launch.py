import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser(description='Launch Gazebo simulation in XTDrone2')

    parser.add_argument('--world', type=str, help='World name', required=False, default="default")
    parser.add_argument('--model-store', type=str, help='Model store path', required=False, default="~/.simulation-gazebo/models:~/.simulation-gazebo/worlds")

    args, unknown = parser.parse_known_args()

    gazebo_cmd = f'GZ_SIM_RESOURCE_PATH={args.model_store} gz sim -r {args.world}.sdf'

    _handle = subprocess.Popen(['bash', '-c', gazebo_cmd])

    try:
        while True:
            if _handle.poll() is not None:
                break
            else:
                _handle.wait()
    except KeyboardInterrupt:
        _handle.terminate()
        exit(0)



if __name__ == '__main__':
    main()