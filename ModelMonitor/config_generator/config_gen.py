import argparse
import json
import yaml
import os
from xml_parser import parse_xml

def generate_monitor_files(scxml_path, monitor_ws, log_path, oracle_address, config_path):
    parsed_data = parse_xml(scxml_path)
    skill_name = parsed_data["skill_name"]
    yaml_filename = os.path.join(config_path, f"monitor_{skill_name}.yaml")
    
    oracle_ip, oracle_port = oracle_address.split(":")
    
    topics = [
        {"name": t["name"], "type": t["type"], "action": "log"}
        for t in parsed_data["monitored"] if t["protocol"] == "topic"
    ] or None
    
    services = [
        {"name": s["name"], "type": s["type"], "action": "log"}
        for s in parsed_data["monitored"] if s["protocol"] == "service"
    ] or None
    
    monitor_config = {
        "path": monitor_ws,
        "monitors": [
            {
                "monitor": {
                    "id": f"monitor_{skill_name}",
                    "log": log_path,
                    "silent": False,
                    "oracle": {
                        "url": oracle_ip,
                        "port": int(oracle_port),
                        "action": "nothing"
                    },
                }
            }
        ]
    }
    
    if topics:
        monitor_config["monitors"][0]["monitor"]["topics"] = topics
    if services:
        monitor_config["monitors"][0]["monitor"]["services"] = services
    
    with open(yaml_filename, "w") as yaml_file:
      yaml.dump(monitor_config, yaml_file, default_flow_style=False, sort_keys=False)
      print(f"YAML file '{yaml_filename}' successfully generated.")
  
    json_filename = os.path.join(config_path, f"{skill_name}_ioshape.json")
    json_data = {"inputs": parsed_data["inputs"], "outputs": parsed_data["outputs"]}
    with open(json_filename, "w") as json_file:
      json.dump(json_data, json_file, indent=4)
      print(f"JSON file '{json_filename}' successfully generated.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a monitor YAML file from an SCXML file.")
    parser.add_argument("scxml_path", help="Path to the SCXML file")
    parser.add_argument("--monitor-ws-path", default=None, help="Path to the monitor workspace (default: ~/monitor_<skill_name>_ws)")
    parser.add_argument("--log-path", default=None, help="Path to the log directory (default: ~/monitor_<skill_name>_ws)")
    parser.add_argument("--oracle-address", required=True, help="Oracle address in the format IP:PORT")
    parser.add_argument("--config-path", default="./", help="Path to store generated files (default: ./)")
    
    args = parser.parse_args()
    skill_name = parse_xml(args.scxml_path)["skill_name"]
    monitor_ws = args.monitor_ws_path or os.path.expanduser(f"~/monitor_{skill_name}_ws")
    log_path = args.log_path or os.path.expanduser(f"~/monitor_{skill_name}_ws")
    generate_monitor_files(args.scxml_path, monitor_ws, log_path, args.oracle_address, args.config_path)
