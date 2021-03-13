package lib.util.drivers;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.Counter;
import lib.drivers.CounterWrapper;

public class CounterWrapperTest {
    
    @Test
    public void test() {
        Counter counter = new Counter(2);
        CounterWrapper testCounter = new CounterWrapper(counter);

        testCounter.setSimPeriod(0.004096);
        assertEquals(0.004096, testCounter.getPeriod(), 0.0001);
    }
}
