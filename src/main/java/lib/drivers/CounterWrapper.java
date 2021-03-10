package lib.drivers;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.RobotBase;

public class CounterWrapper {
    
    private Counter counter;
    private double simPeriod;

    private boolean isReal;

    public CounterWrapper(Counter counter) {
        this.counter = counter;
        this.isReal = RobotBase.isReal(); 
    }

    /**
     * Returns the counter object wrapped inside
     */
    public Counter get() {
        return counter;
    }

    public double getPeriod() {
        if(isReal) {
            return counter.getPeriod();
        } else {
            return simPeriod;
        }
    }

    public void setSimPeriod(double period) {
        simPeriod = period;
    }
}
