package lib.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LatchedBooleanTest {
    LatchedBoolean latch = new LatchedBoolean();

    @Test
    public void test() {
        assertEquals(false, latch.update(false));
        assertEquals(true, latch.update(true));
        assertEquals(false, latch.update(true));
        assertEquals(false, latch.update(true));
        
        assertEquals(false, latch.update(false));
        assertEquals(true, latch.update(true));
        assertEquals(false, latch.update(true));
        assertEquals(false, latch.update(true));
    }
}