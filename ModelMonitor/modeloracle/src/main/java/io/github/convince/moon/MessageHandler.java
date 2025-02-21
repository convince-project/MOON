package io.github.convince.moon;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.json.JSONObject;

public class MessageHandler {
    private ArrayList<IOEntry> inputs;
    private ArrayList<IOEntry> outputs;

    public MessageHandler(ArrayList<IOEntry> inputs, ArrayList<IOEntry> outputs) {
        this.inputs = inputs;
        this.outputs = outputs;
    }

    public Message fromScxmlEvent(String scxmlName, Map<String, Object> payload) throws Exception {
        IOEntry entry = fromScxmlName(scxmlName);

        String name = entry.getEventName();
        String type = entry.getType();

        Map<String, Object> data = new HashMap<>();
        for (String key : payload.keySet()) {
            data.put(key.replaceFirst("^ros_fields__", ""), payload.get(key));
        }

        return new Message(name, type, data);
    }

    public String toScxmlEventName(Message m) throws Exception {
        for (IOEntry entry : this.inputs) {
            if (m.getName().equals(entry.getEventName()) && m.getType().equals(entry.getType())) {
                return entry.getScxmlName();
            }
        }
        throw new Exception("Unexpected message");
    }

    public Message fromJSON(JSONObject jo) throws Exception {
        String name = null;
        String type = null;

        if (jo.keySet().contains("topic")) {
            name = jo.getString("topic");
            type = "topic";
        } else if (jo.keySet().contains("service")) {
            name = jo.getString("service");
            if (jo.keySet().contains("request")) {
                type = "request";
            } else if (jo.keySet().contains("response")){
                type = "response";
            } else {
                throw new Exception("Neither request nor response in service message");
            }
            
        } else {
            throw new Exception("Not a service nor topic message");
        }

        // System.out.println("Parsed event " + name + " " + type);
        // System.out.println(this.inputs.size());

        ArrayList<String> params = this.getParamsFromNameType(name, type);
        HashMap<String, Object> data = new HashMap<>();

        for (String param : params) {
            if (jo.keySet().contains("response")) {
                data.put(param, jo.getJSONObject("response").get(param));
            } else if (jo.keySet().contains("request")) {
                data.put(param, jo.getJSONObject("request").get(param));
            } else if (jo.keySet().contains("topic")) {
                data.put(param, jo.get(param));
            } else {
                throw new Exception("Unknown message type");
            }
        }

        return new Message(name, type, data);
    }

    private ArrayList<String> getParamsFromNameType(String name, String type) throws Exception {
        for (IOEntry entry : this.inputs) {
            // System.out.println(name + " " + entry.getEventName() + " " + type + " " + entry.getType());
            if (name.equals(entry.getEventName()) && type.equals(entry.getType())) {
                // System.out.println("Parsed name " + name + " Found name " + entry.getEventName());
                return entry.getParams();
            }
        }
        for (IOEntry entry : this.outputs) {
            if (name.equals(entry.getEventName()) && type.equals(entry.getType())) {
                return entry.getParams();
            }
        }
        throw new Exception("Event not found");
    }

    private IOEntry fromScxmlName(String scxmlName) throws Exception {
        for (IOEntry entry : this.outputs) {
            if (scxmlName.equals(entry.getScxmlName())) {
                return entry;
            }
        }
        throw new Exception("Unexpected event");
    }

    public boolean isOutput(Message m) throws Exception {
        for (IOEntry entry : this.outputs) {
            if (m.getName().equals(entry.getEventName()) && m.getType().equals(entry.getType())) {
                return true;
            }
        }
        for (IOEntry entry : this.inputs) {
            if (m.getName().equals(entry.getEventName()) && m.getType().equals(entry.getType())) {
                return false;
            }
        }
        throw new Exception("Event not found");
    }
} 
