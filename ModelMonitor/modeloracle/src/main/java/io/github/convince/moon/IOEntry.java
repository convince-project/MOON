package io.github.convince.moon;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;

public class IOEntry {
    private String eventName;
    private String type;
    private String scxmlName;
    private ArrayList<String> params;

    // Constructor to initialize all attributes
    public IOEntry(String eventName, String type, String scxmlName, ArrayList<String> params) {
        this.eventName = eventName;
        this.type = type;
        this.scxmlName = scxmlName;
        this.params = new ArrayList<>(params);
    }

    // Getter methods
    public String getEventName() {
        return eventName;
    }

    public String getType() {
        return type;
    }

    public String getScxmlName() {
        return scxmlName;
    }

    public ArrayList<String> getParams() {
        return new ArrayList<>(params);
    }

    public static IOEntry fromJSONObject(JSONObject jo) {
        String eventName = jo.getString("event_name");
        String type = jo.getString("type");
        String scxmlName = jo.getString("scxml_name");

        ArrayList<String> params = new ArrayList<>();
        JSONArray joParams = jo.getJSONArray("params");
        for (int i=0; i<joParams.length(); i++) {
            params.add(joParams.getString(i));
        }

        return new IOEntry(eventName, type, scxmlName, params);
    }

    public String toString() {
        return "Event name " + this.getEventName() + "\nEvent type " + this.getType();
    }
}