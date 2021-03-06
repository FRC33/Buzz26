package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BarrelRacingPath2 implements PathContainer {

    public BarrelRacingPath2() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        
        sWaypoints.add(new Waypoint(40,90,0,0));
        sWaypoints.add(new Waypoint(165,90,0,120));
        sWaypoints.add(new Waypoint(175,90,5,50));
        sWaypoints.add(new Waypoint(175,80,0,50));
        sWaypoints.add(new Waypoint(175,50,0,120));
        sWaypoints.add(new Waypoint(175,40,5,50));
        sWaypoints.add(new Waypoint(165,40,0,50));
        
        PathBuilder.shiftWaypoints(sWaypoints);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}