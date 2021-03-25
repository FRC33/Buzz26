package frc2020.paths;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class TrajectoryRegistry {

    private static TrajectoryRegistry mInstance;

    private HashMap<String, Trajectory> mTrajectories = new HashMap<>();

    public synchronized static TrajectoryRegistry getInstance() {
        if (mInstance == null) {
            mInstance = new TrajectoryRegistry();
        }

        return mInstance;
    }

    private TrajectoryRegistry() {
        
    }

    public void load(String... names) {
        for(String name : names) {
            load(name);
        }
    }

    /**
     * Loads a path json file generated from PathWeaver intp the registry.
     * @param name The name of the path in PathWeaver
     */
    public void load(String name) {
        String trajectoryJSON = "paths/" + name + ".wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        mTrajectories.put(name, trajectory);
    }

    public Trajectory get(String name) {
        if(mTrajectories.containsKey(name)) {
            return mTrajectories.get(name);
        } else {
            return new Trajectory();
        }
    }
}
