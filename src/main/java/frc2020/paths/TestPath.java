package frc2020.paths;


import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class TestPath implements PathContainer {

    public TestPath() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

        sWaypoints.add(new PathBuilder.Waypoint(0, 0, 0, 100));
        sWaypoints.add(new PathBuilder.Waypoint(30, 0, 0, 100));
        sWaypoints.add(new PathBuilder.Waypoint(50, 0, 10, 50));
        sWaypoints.add(new PathBuilder.Waypoint(50, -20, 0, 100));
        sWaypoints.add(new PathBuilder.Waypoint(50, -50, 0, 100));
        sWaypoints.add(new PathBuilder.Waypoint(50, -60, 0, 0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}