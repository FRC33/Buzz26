package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BouncePath4 implements PathContainer {

    public BouncePath4() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        //Start at 3rd star
        sWaypoints.add(new Waypoint(265,180,0,0));

        //Yeet back into end goal
        sWaypoints.add(new Waypoint(265,125,5,60));
        sWaypoints.add(new Waypoint(300,125,0,60));
        sWaypoints.add(new Waypoint(370,125,0,60));

        PathBuilder.shiftWaypoints(sWaypoints, 40, 90);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}