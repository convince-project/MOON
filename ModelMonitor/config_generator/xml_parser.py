import xml.etree.ElementTree as ET
import re
from collections import defaultdict

def parse_xml(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    namespace = root.tag.split("}")[0].strip("{")
    root_name = root.get("name", "")
    
    def strip_ns(tag):
        return tag.split("}")[-1] if "}" in tag else tag
    
    def clean_param(param):
        return re.sub(r"^_msg\.|^_req\.|^_res\.", "", param) if param else param
    
    def extract_variable(expression):
        match = re.search(r"_msg\.([a-zA-Z0-9_]+)|_req\.([a-zA-Z0-9_]+)|_res\.([a-zA-Z0-9_]+)", expression)
        return match.group(1) or match.group(2) or match.group(3) if match else expression
    
    def translate_name(name, is_service, is_request, is_response):
        parts = name.strip("/").split("/")
        base_name = "__".join(parts)
        prefix = "srv_" if is_service else "topic_"
        suffix = "_request" if is_request else "_response" if is_response else "_msg"
        translated_name = f"{prefix}{base_name}".replace("___", "__")
        client_name = ""
        if parts and parts[0] != root_name:
            client_name = f"_client_{root_name}"
            if is_request:
                suffix = "_req"
        translated_name += f"{suffix}{client_name}"
        return translated_name
    
    monitored = []
    inputs = defaultdict(set)
    outputs = defaultdict(set)
    
    monitored_tags = {
        "ros_service_server": "service",
        "ros_service_client": "service",
        "ros_topic_subscriber": "topic",
        "ros_topic_publisher": "topic",
    }
    
    for elem in root.iter():
        tag = strip_ns(elem.tag)
        if tag in monitored_tags:
            name = elem.get("service_name") if "service_name" in elem.attrib else elem.get("topic")
            if name:
                name = name.strip("/")
            monitored.append({
                "protocol": monitored_tags[tag],
                "name": name,
                "type": ".".join(elem.get("type", "").split("/")[:1] + ["srv" if monitored_tags[tag] == "service" else "msg"] + elem.get("type", "").split("/")[1:])
            })
    
    for elem in root.iter():
        tag = strip_ns(elem.tag)
        if tag in ["ros_service_handle_request", "ros_service_handle_response", "ros_topic_callback"]:
            event_name = elem.get("name")[1:]
            event_type = "request" if "request" in tag else "response" if "response" in tag else "topic"
            is_service = "service" in tag
            is_request = event_type == "request"
            is_response = event_type == "response"
            scxml_name = translate_name(event_name, is_service, is_request, is_response)
            params = [clean_param(field.get("name")) for field in elem.findall(f"{{{namespace}}}field")]
            params += [clean_param(assign.get("expr")) for assign in elem.findall(f"{{{namespace}}}assign")]
            if elem.get("cond"):
                params.append(extract_variable(elem.get("cond")))
            inputs[(event_name, event_type)].update(params)
        elif tag in ["ros_service_send_request", "ros_service_send_response"]:
            event_name = elem.get("name")[1:]
            event_type = "request" if "request" in tag else "response" if "response" in tag else "topic"
            is_service = "service" in tag
            is_request = event_type == "request"
            is_response = event_type == "response"
            scxml_name = translate_name(event_name, is_service, is_request, is_response)
            params = [clean_param(field.get("name")) for field in elem.findall(f"{{{namespace}}}field")]
            params += [clean_param(assign.get("expr")) for assign in elem.findall(f"{{{namespace}}}assign")]
            if elem.get("cond"):
                params.append(extract_variable(elem.get("cond")))
            outputs[(event_name, event_type)].update(params)
    
    inputs = [{"event_name": k[0], "type": k[1], "scxml_name": translate_name(k[0], k[1] == "request" or k[1] == "response", k[1] == "request", k[1] == "response"), "params": list(v)} for k, v in inputs.items()]
    outputs = [{"event_name": k[0], "type": k[1], "scxml_name": translate_name(k[0], k[1] == "request" or k[1] == "response", k[1] == "request", k[1] == "response"), "params": list(v)} for k, v in outputs.items()]
    
    return {"skill_name": root_name, "monitored": monitored, "inputs": inputs, "outputs": outputs}


# Example usage:
# root_name, monitored, inputs, outputs = parse_xml("AlarmSkillHigh.scxml")
# print(root_name)
# print(monitored)
# print(inputs)
# print(outputs)
