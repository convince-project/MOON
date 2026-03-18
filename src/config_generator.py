from enum import Enum
import os
from typing import Dict, List, Optional, Tuple, Union
import argparse
import sys

from dataclasses import dataclass, field

import lxml.etree as ET
from lxml.etree import _Element as XmlElement

from pattern_translator import Pattern, PatternInfo, Scope, translate_pattern

from generator import get_moon_path

from jinja2 import Template


TEMPLATE_DIR=os.path.join(get_moon_path(), 'template')


@dataclass
class Predicate:
    name: str
    field_name: str = ""
    field_type: str = ""
    starting_value: Union[str, int, float, bool] = None
    event: bool = False
    parent_interface: Dict[str, str] = field(default_factory=dict)

class TimeUnit(Enum):
    """List of supported time units."""
    SECONDS = 1
    MILLISECONDS = 1_000
    MICROSECONDS = 1_000_000
    NANOSECONDS = 1_000_000_000

def _string_to_time_unit(time_unit: str) -> TimeUnit:
        assert time_unit in ["s", "ms", "us", "ns"], "Unsupported time unit"
        match time_unit:
            case "s":
                return TimeUnit.SECONDS
            case "ms":
                return TimeUnit.MILLISECONDS
            case "us":
                return TimeUnit.MICROSECONDS
            case "ns":
                return TimeUnit.NANOSECONDS

def _convert_time_to_seconds(time_interval: str, time_unit: TimeUnit) -> float:
        interval_value = float(time_interval)
        time_unit_ratio = float(TimeUnit.SECONDS.value / time_unit.value)
        interval_value = interval_value * time_unit_ratio
        return interval_value

def _transform_field_name(field_name: str) -> str:
    fields = field_name.split('.')
    transformed_fields = []
    for field in fields:
        transformed_fields.append(f'["{field}"]')
    transformed_field_name = "".join(transformed_fields)
    return transformed_field_name

def _process_ports(ports_node: XmlElement) -> Tuple[List[Dict], Optional[Dict[str, str]]]:
    """Process all port declarations in the ports section."""

    interfaces: List[Dict] = []
    clock = None

    for interface in ports_node:
        tag = interface.tag
        assert tag in ["ros_topic", "ros_service", "ros_action", "clock"], "Invalid port tag"

        match tag:
            case "ros_topic":
                interface_name = interface.attrib["topic_name"]
            case "ros_service":
                interface_name = interface.attrib["service_name"]
            case "ros_action":
                interface_name = interface.attrib["action_name"]
            case "clock":
                clock = {
                    "name": interface.attrib["topic_name"],
                    "type": interface.attrib["type"],
                }
                break
        
        tag = tag[4:]

        interface_type = interface.attrib["type"].replace("/", ".")

        predicates = []

        for var in interface:
            assert var.tag in ["state_var", "event_var", "goal_id", "result_code"], "Invalid variable type"

            match var.tag:
                case "state_var":
                    predicates.append(Predicate(
                        var.attrib["id"],
                        _transform_field_name(var.attrib["field"]),
                        var.attrib["type"],
                        var.attrib["expr"],
                        False,
                        parent_interface={
                            "name": interface_name if interface_name.startswith("/") else f"/{interface_name}",
                            "tag": tag,
                            "event": interface.attrib["event"],
                        },
                    ))
                case "event_var":
                    predicates.append(Predicate(
                        var.attrib["id"],
                        event=True,
                        parent_interface={
                            "name": interface_name if interface_name.startswith("/") else f"/{interface_name}",
                            "tag": tag,
                        },
                    ))
                case "result_code":
                    predicates.append(Predicate(
                        var.attrib["id"],
                        parent_interface={
                            "name": interface_name if interface_name.startswith("/") else f"/{interface_name}",
                            "tag": tag,
                        },
                    ))

        interfaces.append({
            "interface_name": interface_name,
            "interface_type": interface_type,
            "interface_tag": tag,
            "predicates": predicates,
        })

    return interfaces, clock

def _extract_variable(event: str, separator: str) -> str:
    if separator in event:
        return event.split(separator)[0].strip()
    return event.strip()

def _predicates_from_event(event: str) -> List[str]:
    predicates = []
    if any(x in event for x in ["&&", "||"]):
        predicates = event.split("&&")
        for p in predicates:
            if "||" in p:
                predicates.remove(p)
                predicates.append(p.split("||"))
    else:
        predicates.append(event)
    
    nps = []
    for p in predicates:
        np = p
        for separator in ["==", ">", ">=", "<", "<="]:
            np = _extract_variable(np, separator)
        nps.append(np)
    return nps

def _process_properties(properties_node: XmlElement) -> List[Dict]:
    properties = []
    for property in properties_node:
        property_id = property.attrib["id"]
        pattern_string = property.attrib["pattern"]
        events = []
        scope = ""
        scope_events = []
        time = None

        for child in property:
            tag = child.tag
            if tag == "event":
                events.append(child.text)
            if tag == "events":
                for event in child:
                    events.append(event.text)
            if tag == "time_interval":
                property_time_unit = _string_to_time_unit(child.attrib["time_unit"])
                if child.attrib.get("time") == None:
                    after = child.attrib.get("after")
                    within = child.attrib.get("within")
                    if after is not None:
                        after = _convert_time_to_seconds(after, property_time_unit)
                    if within is not None:
                        within = _convert_time_to_seconds(within, property_time_unit)
                    time = (after, within)
                else:
                    time = child.attrib["time"]
                    time = _convert_time_to_seconds(time, property_time_unit)
            if tag == "scope":
                scope = child.attrib["type"]
                scope_events.append(child.attrib.get("event"))
        
        match pattern_string:
            case "universality":
                pattern = Pattern.UNIVERSALITY
            case "absence":
                pattern = Pattern.ABSENCE
            case "response":
                pattern = Pattern.RESPONSE
            case "recurrence":
                pattern = Pattern.RECURRENCE
            case "precedence":
                pattern = Pattern.PRECEDENCE
            case "existence":
                pattern = Pattern.EXISTENCE
            case _:
                raise Exception

        match scope:
            case "globally":
                scope = Scope.GLOBALLY
            case "before":
                scope = Scope.BEFORE
            case "after":
                scope = Scope.AFTER
            case _:
                raise Exception
            
        reelay_compatible_events = []
        for event in events:
            reelay_compatible_events.append(event.replace("==", ":"))

        property_pattern = PatternInfo(
            pattern=pattern,
            scope=scope,
            events=reelay_compatible_events,
            scope_events=scope_events,
            time=time,
        )
        translated_property = translate_pattern(property_pattern)

        involved_variables = []
        for event in events:
            new_predicates = _predicates_from_event(event)
            for np in new_predicates:
                if np not in involved_variables:
                    involved_variables.append(np)

        properties.append({
            "name": property_id,
            "statement": translated_property,
            "variables": involved_variables,
            "pattern": pattern_string,
        })

    return properties

def _get_interface_from_predicate(predicate: str, interfaces: Dict) -> Dict:
    for i in interfaces:
        for p in i["predicates"]:
            if p.name == predicate:
                return i
    

def main(argv):
    parser = argparse.ArgumentParser(
        description="Generate MOON compatible configuration files from an XML specification",
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--property-file",
        help="XML property specification file",
        default="./properties.xml",
        metavar="STRING")
    parser.add_argument("--config-out-dir",
        help="Output directory of the generated config files",
        default="./",
        metavar="STRING")
    parser.add_argument("--monitor-workspace",
        help="Monitor workspace to specify for the monitor configuration file",
        metavar="STRING")
    parser.add_argument("--oracle-port",
        help="Port on which the oracle is listening",
        default=8080,)
    args = parser.parse_args()

    xml_parser = ET.XMLParser(remove_comments=True)
    with open(args.property_file, "r", encoding="utf-8") as pf:
        xml_tree = ET.parse(pf, parser=xml_parser)
        root = xml_tree.getroot()
        assert root.tag == "properties", "Invalid root tag"

        ports_node: XmlElement = root.find("ports")
        assert ports_node is not None, "No ports defined"

        assumes_node: XmlElement = root.find("assumes")
        guarantees_node: XmlElement = root.find("guarantees")
        assert (assumes_node is not None or guarantees_node is not None), "No properties defined"

    properties = []

    ports, clock = _process_ports(ports_node)

    if assumes_node is not None:
        properties.extend(_process_properties(assumes_node))
    if guarantees_node is not None:
        properties.extend(_process_properties(guarantees_node))

    oracle_port = args.oracle_port

    with open(f"{TEMPLATE_DIR}/monitor_config.yaml.j2", "r") as f:
        monitor_template = Template(f.read(), trim_blocks=True, lstrip_blocks=True)

    with open(f"{TEMPLATE_DIR}/property.py.j2", "r") as f:
        property_template = Template(f.read(), trim_blocks=True, lstrip_blocks=True)

    for property in properties:
        relevant_interfaces = []
        predicates = []
        for v in property["variables"]:
            relevant_interface = _get_interface_from_predicate(v, ports)
            if relevant_interface not in relevant_interfaces:
                relevant_interfaces.append(relevant_interface)
            for p in relevant_interface["predicates"]:
                if p.name == v:
                    predicates.append(p)

        topics = []
        for i in relevant_interfaces:
            if i["interface_tag"] == "topic":
                topics.append({
                    "name": i["interface_name"],
                    "type": i["interface_type"],
                    "qos": "reliable",
                })
        if property["pattern"] == "recurrence":
            topics.append({
                "name": clock["name"],
                "type": clock["type"],
                "qos": "reliable",
            })
        services = []
        for i in relevant_interfaces:
            if i["interface_tag"] == "service":
                services.append({
                    "name": i["interface_name"],
                    "type": i["interface_type"],
                })
        actions = []
        for i in relevant_interfaces:
            if i["interface_tag"] == "action":
                actions.append({
                    "name": i["interface_name"],
                    "type": i["interface_type"],
                })

        monitor_config = monitor_template.render(
            monitor_workspace=args.monitor_workspace,
            monitor_name=f"{property["name"]}_monitor",
            oracle_port=oracle_port,
            topics=topics,
            services=services,
            actions=actions,
        )
        with open(os.path.join(args.config_out_dir, f"{property["name"]}_monitor_config.yaml"), "w") as f:
            f.write(monitor_config)
            print(f"Generated monitor configuration file: {property["name"]}_monitor_config.yaml")

        property_config = property_template.render(
            property=property["statement"],
            predicates=predicates,
            pattern=property["pattern"],
            actions=len(actions) != 0,
        )
        with open(os.path.join(args.config_out_dir, f"{property["name"]}.py"), "w") as f:
            f.write(property_config)
            print(f"Generated property file: {property["name"]}.py")

        oracle_port += 1


if __name__ == "__main__":
    main(sys.argv[1:])
