package io.github.convince.moon;

// WebSocket
import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.JSONArray;
// JSON
import org.json.JSONObject;


public class Oracle extends WebSocketServer {

    private MessageHandler messageHandler;

    private StateMachine stateMachine;

    private ArrayList<Message> monitoredMessages = new ArrayList<>();
    private ArrayList<Message> producedMessages = new ArrayList<>();


    public Oracle(InetSocketAddress address, ArrayList<IOEntry> inputs, ArrayList<IOEntry> outputs, StateMachine stateMachine) {
        super(address);
        this.messageHandler = new MessageHandler(inputs, outputs);
        this.stateMachine = stateMachine;
    }

    public ArrayList<Message> getMonitoredMessages() {
        return this.monitoredMessages;
    }

    public ArrayList<Message> getProducedMessages() {
        return this.producedMessages;
    }

    public boolean evaluate() {
        // System.out.println(monitoredMessages);
        // System.out.println(producedMessages);
        if (monitoredMessages.equals(producedMessages)) {
            return true;
        }
        
        int minSize = Math.min(monitoredMessages.size(), producedMessages.size());
        
        for (int i = 0; i < minSize; i++) {
            if (!monitoredMessages.get(i).equals(producedMessages.get(i))) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        // New Client
        //conn.send("Welcome to the server!"); // This method sends a message to the new client
        //broadcast("new connection: " + handshake.getResourceDescriptor()); // This method sends a message to all clients connected
        System.out.println("new connection to " + conn.getRemoteSocketAddress());
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        // Client Left
        System.out.println(
                "closed " + conn.getRemoteSocketAddress() + " with exit code " + code + " additional info: " + reason);
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        // Message Received
        System.out.println("received message from " + conn.getRemoteSocketAddress() + ": " + message);
        JSONObject jo = new JSONObject(message);
        Message m = null;
        try {
            m = this.messageHandler.fromJSON(jo);
        } catch (Exception e) {
            System.out.println("Couldn't parse received message " + e.getMessage());
            System.exit(1);
        }

        // System.out.println(m.getData());
        // System.out.println(m.getPayload());

        try {
            if (this.messageHandler.isOutput(m)) {
                this.monitoredMessages.add(m);
            } else {
                Map<String, Object> payload = (m.getData().size() == 0) ? null : m.getPayload();
                System.out.println("Event_name: " + this.messageHandler.toScxmlEventName(m) + " payload " + payload);
                this.stateMachine.triggerEvent(this.messageHandler.toScxmlEventName(m), payload);
                
                Map<String, Object> dispatchedEvent = this.stateMachine.getDispatcher().waitForEvent(100);
                String dispatchedEventName = (String) dispatchedEvent.keySet().toArray()[0];
                Object dispatchedPayload = dispatchedEvent.values().toArray()[0];
                Map<String, Object> dispatchedEventPayload = (dispatchedPayload == null) ? null : (Map<String, Object>) dispatchedPayload;
                m = this.messageHandler.fromScxmlEvent(dispatchedEventName, dispatchedEventPayload);
                this.producedMessages.add(m);
            }
        } catch (Exception e) {
            System.out.println("Unexpected message " + e.getMessage());
        }

        String verdict = null;
        try {
            System.out.println(monitoredMessages);
            System.out.println(producedMessages);
            verdict = (this.evaluate()) ? "currently_true" : "currently_false";
        } catch (Exception e) {
            System.out.println("Error in evaluation " + e.getMessage());
        }

        jo.put("verdict", verdict);
        jo.put("spec", "placeholder");
        jo.put("time", jo.getDouble("time")); // Needed to prevent conversion to int

        conn.send(jo.toString());
        System.out.println(jo.toString());
        System.out.println(verdict);

        System.out.println();
    }

    @Override
    public void onMessage(WebSocket conn, ByteBuffer message) {
        // Message Received
        System.out.println("received ByteBuffer from " + conn.getRemoteSocketAddress());
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        System.err.println("an error occurred on connection " + conn.getRemoteSocketAddress() + ":" + ex);
    }

    @Override
    public void onStart() {
        System.out.println("server started successfully");
    }

    @Override
    public void run() {
        try {
            this.stateMachine.start();
        } catch (Exception e) {
            System.out.println("Could not start state machine " + e.getMessage());
        }
        super.run();
    }

    public static void main(String[] args) {
        String host = "127.0.0.1"; // Default IP
        int port = 3377; // Default Port
        String ioShapePath = null;
        String stateMachinePath = null;

        ArrayList<IOEntry> inputs = new ArrayList<>();
        ArrayList<IOEntry> outputs = new ArrayList<>();

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--address":
                    if (i + 1 < args.length) {
                        String[] addressParts = args[i + 1].split(":");
                        if (addressParts.length == 2) {
                            host = addressParts[0];
                            try {
                                port = Integer.parseInt(addressParts[1]);
                            } catch (NumberFormatException e) {
                                System.out.println("Error: Invalid port number in --address argument.");
                                System.exit(1);
                            }
                        } else {
                            System.out.println("Error: --address requires a value in the format ip:port.");
                            System.exit(1);
                        }
                        i++; // Skip next argument
                    } else {
                        System.out.println("Error: --address requires a value.");
                        System.exit(1);
                    }
                    break;
                case "--ioshape":
                    if (i + 1 < args.length) {
                        ioShapePath = args[i + 1];
                        if (!new File(ioShapePath).exists()) {
                            System.out.println("Error: JSON file does not exist.");
                            System.exit(1);
                        }
                        i++;
                    } else {
                        System.out.println("Error: --ioshape requires a file path.");
                        System.exit(1);
                    }
                    break;
                case "--statemachine":
                    if (i + 1 < args.length) {
                        stateMachinePath = args[i + 1];
                        if (!new File(stateMachinePath).exists()) {
                            System.out.println("Error: SCXML file does not exist.");
                            System.exit(1);
                        }
                        i++;
                    } else {
                        System.out.println("Error: --statemachine requires a file path.");
                        System.exit(1);
                    }
                    break;
                default:
                    System.out.println("Unknown argument: " + args[i]);
                    System.exit(1);
            }
        }

        // Check if mandatory arguments are provided
        if (ioShapePath == null) {
            System.out.println("Error: --ioshape argument is required.");
            System.exit(1);
        }
        if (stateMachinePath == null) {
            System.out.println("Error: --statemachine argument is required.");
            System.exit(1);
        }

        try {
            String content = new String(Files.readAllBytes(Paths.get(ioShapePath)));
            JSONObject jsonObject = new JSONObject(content);
            JSONArray joInputs = jsonObject.getJSONArray("inputs");
            for (int i=0; i<joInputs.length(); i++) {
                inputs.add(IOEntry.fromJSONObject(joInputs.getJSONObject(i)));
            }
            JSONArray joOutputs = jsonObject.getJSONArray("outputs");
            for (int i=0; i<joOutputs.length(); i++) {
                outputs.add(IOEntry.fromJSONObject(joOutputs.getJSONObject(i)));
            }
        } catch (IOException e) {
            System.out.println("Error reading the file: " + e.getMessage());
            System.exit(1);
        } catch (org.json.JSONException e) {
            System.out.println("Error parsing JSON: " + e.getMessage());
            System.exit(1);
        }

        StateMachine sm = null;
        try {
            sm = new StateMachine(stateMachinePath);
        } catch (Exception e) {
            System.out.println("State machine could not be created:" + e.getMessage());
            System.exit(1);
        }

        System.out.println(inputs);
        System.out.println(outputs);

        Oracle server = new Oracle(new InetSocketAddress(host, port), inputs, outputs, sm);
        server.setConnectionLostTimeout(0);

        Timer timer = new Timer();
        TimerTask task = new PeriodicChecker(server); 

        timer.schedule(task, 1000, 1000);

        server.run();
    }

}
