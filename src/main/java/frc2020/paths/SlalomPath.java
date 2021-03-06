package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class SlalomPath implements PathContainer {

    public SlalomPath() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        //Starting point
        sWaypoints.add(new Waypoint(42,30,0,0));

        //Turn 1
        sWaypoints.add(new Waypoint(70,30,10,85));
        sWaypoints.add(new Waypoint(90,60,0,85)); //Center point (D3)
        sWaypoints.add(new Waypoint(110,90,10,85));

        //Straight
        sWaypoints.add(new Waypoint(130,90,0,85));
        sWaypoints.add(new Waypoint(180,90,0,120));

        //Turn 2
        sWaypoints.add(new Waypoint(250,90,10,120));
        sWaypoints.add(new Waypoint(270,60,0,85));
        sWaypoints.add(new Waypoint(290,30,5,85));

        //Turn 3 (full circle)
        sWaypoints.add(new Waypoint(320,30,5,85));
        sWaypoints.add(new Waypoint(340,60,5,85));
        sWaypoints.add(new Waypoint(320,90,5,85));
        sWaypoints.add(new Waypoint(300,90,0,85));
        sWaypoints.add(new Waypoint(280,90,5,85));
        //Getting out
        sWaypoints.add(new Waypoint(270,60,0,85));

        //Straight
        sWaypoints.add(new Waypoint(250,30,5,100));
        sWaypoints.add(new Waypoint(180,30,0,100));
        sWaypoints.add(new Waypoint(130,30,0,85));

        //Turn 4
        sWaypoints.add(new Waypoint(110,30,10,85));
        sWaypoints.add(new Waypoint(90,60,0,85)); //Center point (D3)
        sWaypoints.add(new Waypoint(70,90,10,85));
        sWaypoints.add(new Waypoint(50,90,0,85));
        
        PathBuilder.shiftWaypoints(sWaypoints);

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}