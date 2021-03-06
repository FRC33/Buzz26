package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class GalacticSearchPathRedA implements PathContainer {

    public GalacticSearchPathRedA() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(47,103,0,0));
        sWaypoints.add(new Waypoint(70,103,8,100));
        sWaypoints.add(new Waypoint(90,90,5,100));
        sWaypoints.add(new Waypoint(150,60,0,100));
        sWaypoints.add(new Waypoint(170,50,10,100));
        sWaypoints.add(new Waypoint(165,137,3,100));
        sWaypoints.add(new Waypoint(170,147,2,60));
        sWaypoints.add(new Waypoint(200,155,2,70));
        sWaypoints.add(new Waypoint(340,147,0,150));
        sWaypoints.add(new Waypoint(350,147,0,150));

        PathBuilder.shiftWaypoints(sWaypoints);

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}