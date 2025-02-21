package io.github.convince.moon;

import java.util.HashMap;
import java.util.Map;

public class Message {
    String name;
    String type;
    Map<String, Object> data;

    public Message(String name, String type, Map<String, Object> data) {
        this.name = name;
        this.type = type;
        this.data = data;
    }

    public String getName() {
        return this.name;
    }

    public String getType() {
        return this.type;
    }

    public Map<String, Object> getData() {
        return this.data;
    }

    public Map<String, Object> getPayload() {
        Map<String, Object> payload = new HashMap<>();
        for (String key : this.data.keySet()) {
            payload.put("ros_fields__" + key, this.data.get(key));
        }
        return payload;
    }

    public String toString() {
        return "Name: " + this.name + " Type: " + this.type + " Data: " + this.data;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Message that = (Message) o;
        return this.name.equals(that.name) && this.type.equals(that.type) && this.data.equals(that.data);
    }
}
