package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BouncePath2 implements PathContainer {

    public BouncePath2() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        //Start at star
        sWaypoints.add(new Waypoint(85,140,0,0));

        //To next turn
        sWaypoints.add(new Waypoint(85,110,10,80));
        sWaypoints.add(new Waypoint(130,50,0,100));
        
        //Turn tight
        sWaypoints.add(new Waypoint(150,30,5,100));
        sWaypoints.add(new Waypoint(180,50,5,65));

        //Go straight to next star
        sWaypoints.add(new Waypoint(180,100,0,80));
        sWaypoints.add(new Waypoint(180,110,0,100));
        sWaypoints.add(new Waypoint(180,150,0,100));

        PathBuilder.shiftWaypoints(sWaypoints, 40, 90);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}