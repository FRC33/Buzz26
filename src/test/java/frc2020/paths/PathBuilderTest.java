package frc2020.paths;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import frc2020.paths.PathBuilder.Waypoint;

public class PathBuilderTest {

    @Test
    public void testShiftWaypoints() {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(10, 20, 0, 0));
        waypoints.add(new Waypoint(50, 70, 0, 50));

        PathBuilder.shiftWaypoints(waypoints);

        assertEquals(0, waypoints.get(0).position.x(), 0);
        assertEquals(0, waypoints.get(0).position.y(), 0);

        assertEquals(50-10, waypoints.get(1).position.x(), 0);
        assertEquals(70-20, waypoints.get(1).position.y(), 0);
    }
}
