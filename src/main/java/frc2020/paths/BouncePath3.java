package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BouncePath3 implements PathContainer {

    public BouncePath3() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        //Move back from 2nd star
        sWaypoints.add(new Waypoint(180,150,0,0));
        sWaypoints.add(new Waypoint(180,60,10,100));

        //Turn around back
        sWaypoints.add(new Waypoint(200,60,5,80));
        sWaypoints.add(new Waypoint(240,60,1,80));

        //Move to 3rd star
        sWaypoints.add(new Waypoint(265,50,5,80));
        sWaypoints.add(new Waypoint(265,180,0,80));

        PathBuilder.shiftWaypoints(sWaypoints, 40, 90);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}