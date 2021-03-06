package frc2020.paths;

import java.util.ArrayList;

import frc2020.paths.PathBuilder.Waypoint;
import lib.control.Path;

public class BarrelRacingPath1 implements PathContainer {

    public BarrelRacingPath1() {
        
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        //Into first turn
        sWaypoints.add(new Waypoint(40,90,0,0));
        sWaypoints.add(new Waypoint(190,90,30,80));
        sWaypoints.add(new Waypoint(190,60,0,80));
        sWaypoints.add(new Waypoint(190,30,30,80));

        //Out of first turn
        sWaypoints.add(new Waypoint(140,30,0,80));
        sWaypoints.add(new Waypoint(110,30,20,80));
        sWaypoints.add(new Waypoint(110,60,0,80));
        sWaypoints.add(new Waypoint(110,90,20,80));
        
        //Forward
        sWaypoints.add(new Waypoint(190,90,0,80));
        sWaypoints.add(new Waypoint(200,90,0,80));
        sWaypoints.add(new Waypoint(250,90,0,80));
        
        //Into second turn
        sWaypoints.add(new Waypoint(270,120,0,80));
        sWaypoints.add(new Waypoint(270,155,20,80));
        sWaypoints.add(new Waypoint(240,155,0,80));
        sWaypoints.add(new Waypoint(210,155,20,80));


        //Out of second turn
        sWaypoints.add(new Waypoint(210,120,0,80));
        //sWaypoints.add(new Waypoint(210,40,20,100));
        
        //Forward
        sWaypoints.add(new Waypoint(240,50,20,80));
        sWaypoints.add(new Waypoint(270,40,5,80));
        sWaypoints.add(new Waypoint(300,40,0,80));
        
        //Third turn
        sWaypoints.add(new Waypoint(330,40,20,80));
        sWaypoints.add(new Waypoint(330,70,0,80));
        sWaypoints.add(new Waypoint(330,90,15,80));

        //Straight back
        sWaypoints.add(new Waypoint(290,90,0,80));
        
        //sWaypoints.add(new Waypoint(280,70,10,90));
        
        //sWaypoints.add(new Waypoint(270,90,0,90));
        sWaypoints.add(new Waypoint(270,90,0,80));
        sWaypoints.add(new Waypoint(-15,90,0,80));
        sWaypoints.add(new Waypoint(-30,90,0,80));
        
        PathBuilder.shiftWaypoints(sWaypoints);
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}