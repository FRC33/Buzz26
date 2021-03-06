package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class GalacticSearchPathBlueA implements PathContainer {

    public GalacticSearchPathBlueA() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(47,43,0,0));
        sWaypoints.add(new Waypoint(80,43,3,80));
        sWaypoints.add(new Waypoint(130,35,0,80));
        sWaypoints.add(new Waypoint(180,25,5,100));
        sWaypoints.add(new Waypoint(185,120,5,70));
        sWaypoints.add(new Waypoint(210,120,0,80));
        sWaypoints.add(new Waypoint(270,90,5,100));
        sWaypoints.add(new Waypoint(330,90,0,100));
        sWaypoints.add(new Waypoint(350,90,0,100));

        PathBuilder.shiftWaypoints(sWaypoints);

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}