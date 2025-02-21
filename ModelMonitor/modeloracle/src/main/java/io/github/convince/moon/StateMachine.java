package io.github.convince.moon;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Collections;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.apache.commons.scxml2.SCXMLExecutor;
import org.apache.commons.scxml2.SCXMLIOProcessor;
import org.apache.commons.scxml2.TriggerEvent;
import org.apache.commons.scxml2.io.SCXMLReader;
import org.apache.commons.scxml2.EventBuilder;
import org.apache.commons.scxml2.EventDispatcher;
import org.apache.commons.scxml2.model.ModelException;
import org.apache.commons.scxml2.model.SCXML;

public class StateMachine {

    private SCXMLExecutor stateMachine;

    public StateMachine(String path) throws Exception {
        SCXMLExecutor stateMachine = new SCXMLExecutor();

        InputStream scxmlStream = new FileInputStream(new File(path));
        SCXML scxml = SCXMLReader.read(scxmlStream);
        stateMachine.setStateMachine(scxml);
        
        stateMachine.setEventdispatcher(new Dispatcher());

        this.stateMachine = stateMachine;
    }

    public Dispatcher getDispatcher() {
        return (Dispatcher) this.stateMachine.getEventdispatcher();
    }

    public void start() throws ModelException {
        // Start the state machine
        this.stateMachine.go();
    }

    public void triggerEvent(String event, Map<String, Object> payload) throws ModelException {
        TriggerEvent te = null;
        if (payload == null) {
            te = new EventBuilder(event, TriggerEvent.SIGNAL_EVENT).build();
        } else {
            te = new EventBuilder(event, TriggerEvent.SIGNAL_EVENT).data(payload).build();
        }
        this.stateMachine.triggerEvent(te);
    }

    class Dispatcher implements EventDispatcher {
        private final BlockingQueue<Map<String, Object>> dispatchedEvents = new LinkedBlockingQueue<>();

        @Override
        public void send(Map<String, SCXMLIOProcessor> ioProcessors, String id, String target, String type, String event, Object data, Object hints, long delay) {
            System.out.println("Event sent: " + event + data);
            dispatchedEvents.offer(Collections.singletonMap(event, data));
        }

        @Override
        public void cancel(String sendId) {
            // Not needed
        }

        @Override
        public EventDispatcher newInstance() {
            return new Dispatcher();
        }

        public Map<String, Object> waitForEvent(long timeoutSeconds) throws InterruptedException {
            return dispatchedEvents.poll(timeoutSeconds, TimeUnit.SECONDS);
        }
    }

    public static void main(String[] args) {
        StateMachine sm = null;
        try {
            sm = new StateMachine("/home/k/convince/MOON/ModelMonitor/modeloracle/src/main/java/io/github/convince/moon/AlarmSkill.scxml");
            sm.start();
            // sm.triggerEvent("topic_BatteryComponent__battery_level_msg", Collections.singletonMap("ros_fields__data", 40));
            // sm.triggerEvent("srv_BatteryLevelSkill__tick_request", null);

            sm.triggerEvent("srv_AlarmSkill__tick_request", null);
            
            Map<String, Object> dispatchedEvent = sm.getDispatcher().waitForEvent(5);
            System.out.println("Synchronously received dispatched event: " + dispatchedEvent.keySet());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
