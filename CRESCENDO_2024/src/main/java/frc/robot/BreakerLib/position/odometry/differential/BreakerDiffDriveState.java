package frc.robot.BreakerLib.position.odometry.differential;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/** State of a {@link BreakerLegacyDiffDrive} with speeds and distances. */
public class BreakerDiffDriveState {

    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private double leftDriveDistanceMeters;
    private double rightDriveDistanceMeters;

    /**
     * Create a {@link BreakerDiffDriveState} from drivetrain speeds and distances.
     * 
     * @param leftDriveSpeed Left wheel speeds, m/s.
     * @param rightDriveSpeed Right wheel speeds, m/s.
     * @param leftDriveDistance Distance traveled by left wheels, m.
     * @param rightDriveDistance Distance traveled by right wheels, m.
     */
    public BreakerDiffDriveState(double leftDriveSpeed, double rightDriveSpeed, double leftDriveDistance, double rightDriveDistance) {
        wheelSpeeds = new DifferentialDriveWheelSpeeds(leftDriveSpeed, rightDriveSpeed);
        this.leftDriveDistanceMeters = leftDriveDistance;
        this.rightDriveDistanceMeters = rightDriveDistance;
    }

    /**
     * Create a {@link BreakerDiffDriveState} from {@link DifferentialDriveWheelSpeeds} and distances.
     * 
     * @param wheelSpeeds Speeds of drivetrain wheels.
     * @param leftDriveDistance Distance traveled by left wheels, m.
     * @param rightDriveDistance Distance traveled by right wheels, m.
     */
    public BreakerDiffDriveState(DifferentialDriveWheelSpeeds wheelSpeeds, double leftDriveDistance, double rightDriveDistance) {
        this.wheelSpeeds = wheelSpeeds;
        this.leftDriveDistanceMeters = leftDriveDistance;
        this.rightDriveDistanceMeters = rightDriveDistance;
    }

    public double getLeftDriveDistanceMeters() {
        return leftDriveDistanceMeters;
    }

    public double getRightDriveDistanceMeters() {
        return rightDriveDistanceMeters;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return wheelSpeeds;
    }
}
