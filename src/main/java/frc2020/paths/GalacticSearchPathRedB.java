package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class GalacticSearchPathRedB implements PathContainer {

    public GalacticSearchPathRedB() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(47,103,0,0));
        sWaypoints.add(new Waypoint(60,103,4,100));
        sWaypoints.add(new Waypoint(80,115,0,100));
        sWaypoints.add(new Waypoint(95,125,3,100));
        sWaypoints.add(new Waypoint(105,110,0,85));
        sWaypoints.add(new Waypoint(140,70,0,100));
        sWaypoints.add(new Waypoint(155,55,3,100));
        sWaypoints.add(new Waypoint(190,70,5,85));
        sWaypoints.add(new Waypoint(207,120,5,85));
        sWaypoints.add(new Waypoint(227,140,5,85));
        sWaypoints.add(new Waypoint(340,140,0,150));
        sWaypoints.add(new Waypoint(350,140,0,150));

        PathBuilder.shiftWaypoints(sWaypoints);

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}