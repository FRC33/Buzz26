package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BarrelRacingPath3 implements PathContainer {

    public BarrelRacingPath3() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        
        PathBuilder.shiftWaypoints(sWaypoints);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}