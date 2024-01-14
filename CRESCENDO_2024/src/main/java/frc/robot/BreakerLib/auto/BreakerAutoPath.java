package frc.robot.BreakerLib.auto;

import edu.wpi.first.wpilibj2.command.Command;

/** Represnets a Auto Path, contans the path's base {@link Command} and its name */
public class BreakerAutoPath {

    protected Command autoPath;
    protected String pathName;

    /** Creates a BreakerAutoPath.
     * 
     * @param pathName Name of the path.
     * @param autoPath Autopath command.
     */
    public BreakerAutoPath(String pathName, Command autoPath) {
        this.autoPath = autoPath;
        this.pathName = pathName;
    }


    protected BreakerAutoPath() {}

    public String getPathName() {
        return pathName;
    }

    public Command getBaseAutoPath() {
        return autoPath;
    }

    /** Schedules base auto path. */
    public Command startPath() {
        autoPath.schedule();
        return autoPath;
    }
}