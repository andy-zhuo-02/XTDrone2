import argparse
import subprocess

def main():
    parser = argparse.ArgumentParser(description='Launch px4 sitl in XTDrone2')

    parser.add_argument('--model', type=str, help='Model name', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id', required=True)
    parser.add_argument('--x', type=float, help='X position', required=False, default=0)
    parser.add_argument('--y', type=float, help='Y position', required=False, default=0)
    parser.add_argument('--z', type=float, help='Z position', required=False, default=0)
    parser.add_argument('--roll', type=float, help='Roll', required=False, default=0)
    parser.add_argument('--pitch', type=float, help='Pitch', required=False, default=0)
    parser.add_argument('--yaw', type=float, help='Yaw', required=False, default=0)
    parser.add_argument('--world', type=str, help='World name', required=False, default="default")
    parser.add_argument('--px4-dir', type=str, help='PX4 directory', required=False, default="~/PX4-Autopilot")
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")

    args, unknown = parser.parse_known_args()

    autostart_map = {"gz_x500": 4001, "gz_x500_depth":4002}
    sys_autostart = autostart_map[args.model]

    mav_sys_id = args.id + 1

    if args.namespace:
        ns = args.namespace
    else:
        if args.model.startswith("gz_"):
            ns = f"{args.model[3:]}_{args.id}"
        else:
            ns = f"{args.model}_{args.id}"


    px4_cmd = f"PX4_UXRCE_DDS_NS={ns} PX4_GZ_WORLD={args.world} PX4_SYS_AUTOSTART={sys_autostart} PX4_SIM_MODEL={args.model} PX4_GZ_MODEL_POSE='{args.x},{args.y},{args.z},{args.roll},{args.pitch},{args.yaw}' PX4_GZ_STANDALONE=1 {args.px4_dir}/build/px4_sitl_default/bin/px4 -d -s {args.px4_dir}/build/px4_sitl_default/etc/init.d-posix/rcS {args.px4_dir}/ROMFS/px4fmu_common -i {args.id} -w {args.px4_dir}/build/px4_sitl_default"

    _handle = subprocess.Popen(['bash', '-c', px4_cmd])

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