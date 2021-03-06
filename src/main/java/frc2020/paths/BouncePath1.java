package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BouncePath1 implements PathContainer {

    public BouncePath1() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        
        sWaypoints.add(new Waypoint(40,90,0,0));
        sWaypoints.add(new Waypoint(55,90,5,145));
        sWaypoints.add(new Waypoint(75,110,10,145));
        sWaypoints.add(new Waypoint(85,140,0,90));
        
        PathBuilder.shiftWaypoints(sWaypoints);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}