package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class GalacticSearchPathBlueB implements PathContainer {

    public GalacticSearchPathBlueB() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(47,43,0,0));
        sWaypoints.add(new Waypoint(80,43,1,100));
        sWaypoints.add(new Waypoint(100,53,1,100));
        sWaypoints.add(new Waypoint(200,53,7,100));
        sWaypoints.add(new Waypoint(240,120,0,100));
        sWaypoints.add(new Waypoint(260,140,10,100));
        sWaypoints.add(new Waypoint(280,100,0,100));
        sWaypoints.add(new Waypoint(305,50,10,70));
        sWaypoints.add(new Waypoint(325,50,0,150));
        sWaypoints.add(new Waypoint(345,50,0,60));

        PathBuilder.shiftWaypoints(sWaypoints);

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}