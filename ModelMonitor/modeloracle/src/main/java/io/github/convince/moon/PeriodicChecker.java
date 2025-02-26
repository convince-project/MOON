package io.github.convince.moon;

import java.util.TimerTask;

public class PeriodicChecker extends TimerTask {

    private Oracle oracle;
    private int lastMonitoredSize = 0;
    private int lastProducedSize = 0;
    private int attempts = 0;

    private static final int maxAttempts = 5;

    public PeriodicChecker(Oracle oracle) {
        this.oracle = oracle;
    }

    private boolean empiricallyEvaluate() {
        this.lastMonitoredSize = this.oracle.getMonitoredMessages().size();
        this.lastProducedSize = this.oracle.getProducedMessages().size();
        if (this.lastMonitoredSize == this.lastProducedSize) {
            attempts = 0;
        } else {
            attempts++;
        }
        if (this.oracle.evaluate() 
            && Math.abs(this.lastMonitoredSize - this.lastProducedSize) < 5
            && attempts < maxAttempts) {
            return true;
        }
        return false;
    }

    @Override
    public void run() {
        if (empiricallyEvaluate()) {
            System.out.println("Currently True");
        } else if (this.oracle.evaluate()) {
            System.out.println("Timed out");
        } else {
            System.out.println("False");
        }
    }
    
}
