package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class SearchForwardPath2 implements PathContainer {

    public SearchForwardPath2() {

    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        
        sWaypoints.add(new Waypoint(0, 0, 0, 150));
        sWaypoints.add(new Waypoint(130, 0, 0, 150));
       
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}