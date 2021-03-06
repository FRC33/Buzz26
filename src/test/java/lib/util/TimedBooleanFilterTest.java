package lib.util;

import lib.util.TimeBooleanFilter;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;

import org.junit.Test;

public class TimedBooleanFilterTest {
    TimeBooleanFilter filter = new TimeBooleanFilter();

    @Test
    public void test() throws InterruptedException {
        //Starts out false
        filter.update(false, 1.0);
        assertFalse(filter.update(true, 1.0));
        //Rising, still false
        Thread.sleep(500);
        assertFalse(filter.update(true, 1.0));
        //Finishes rising, true
        Thread.sleep(500);
        assertTrue(filter.update(true, 1.0));
        //Starts falling, still true
        assertTrue(filter.update(false, 1.0));
        //Continues falling, still true
        Thread.sleep(500);
        assertTrue(filter.update(false, 1.0));
        //Finishes falling, false
        Thread.sleep(500);
        assertFalse(filter.update(false, 1.0));
    }
}
