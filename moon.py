#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys

def run_command(command, cwd=None, shell=True):
    try:
        subprocess.run(command, cwd=cwd, shell=shell, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command failed with error: {e}")
        sys.exit(e.returncode)

def handle_generate(args):
    target_dir = os.path.join("ROSMonitoring", "generator", "ros2_devel")
    if not os.path.isdir(target_dir):
        print(f"Directory not found: {target_dir}")
        sys.exit(1)

    command = f"python3 generator --config_file {os.path.abspath(args.config_file)}"
    command += f" --out_dir {args.out_dir}"
    command += f'; bash -c "cd {args.out_dir}; source /opt/ros/humble/setup.bash; colcon build"'

    run_command(command, cwd=target_dir)

def handle_run_monitor(args):
    if not os.path.isdir(args.dir):
        print(f"Directory not found: {args.dir}")
        sys.exit(1)

    # Source setup.bash and launch ros2
    command = "bash -c 'source install/setup.bash && ros2 launch monitor/launch/monitor.launch'"
    run_command(command, cwd=args.dir)

def handle_run_oracle(args):
    target_dir = os.path.join("ROSMonitoring", "oracle", "TLOracle")
    if not os.path.isdir(target_dir):
        print(f"Directory not found: {target_dir}")
        sys.exit(1)

    command = f"cp {os.path.abspath(args.property)} {os.path.abspath(target_dir)};"
    prop_no_suffix = args.property.split('.')[0]
    command += f"python3 oracle.py --port {args.port} --online --dense --property {prop_no_suffix}"
    run_command(command, cwd=target_dir)

def main():
    parser = argparse.ArgumentParser(prog="moon")
    subparsers = parser.add_subparsers(dest="command")

    gen_parser = subparsers.add_parser("generate")
    gen_parser.add_argument("--config-file", required=True, help="Path to YAML config file")
    gen_parser.add_argument("--out-dir", required=True, help="Output directory")
    gen_parser.set_defaults(func=handle_generate)

    monitor_parser = subparsers.add_parser("run_monitor")
    monitor_parser.add_argument("--dir", required=True, help="Directory to run monitor from")
    monitor_parser.set_defaults(func=handle_run_monitor)

    oracle_parser = subparsers.add_parser("run_oracle")
    oracle_parser.add_argument("--property", required=True, help="Python file with oracle property")
    oracle_parser.add_argument("--port", required=True, help="Port number to use")
    oracle_parser.set_defaults(func=handle_run_oracle)

    args = parser.parse_args()

    if not hasattr(args, 'func'):
        parser.print_help()
        sys.exit(1)

    args.func(args)

if __name__ == "__main__":
    main()
