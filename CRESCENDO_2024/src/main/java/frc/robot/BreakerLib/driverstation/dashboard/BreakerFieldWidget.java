// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.dashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Static wrapper for Field2d dashboard field widget. */
public class BreakerFieldWidget extends SubsystemBase {
    private static BreakerGenericOdometer odometer;
    private static Field2d field = new Field2d();

    /**
     * Creates field widget.
     * 
     * @param odometer Odometer to use to update robot position.
     */
    public BreakerFieldWidget(BreakerGenericOdometer odometer) {
        BreakerDashboard.getMainTab().add(field);
        BreakerFieldWidget.odometer = odometer;
    }

    /** @return Robot object on field widget. */
    public static FieldObject2d getRobotObject() {
        return field.getRobotObject();
    }

    /**
     * Creates or gets field object with given name.
     * 
     * @param name Name of field object
     * @return Field object with given name
     */
    public static FieldObject2d getFieldObject(String name) {
        return field.getObject(name);
    }

    /**
     * Sets widget poses for given {@link FieldObject2d}.
     * 
     * @param name Name of field objects.
     * @param poses Poses for field objects. Max of 85.
     */
    public static void setFieldObjectPoses(String name, Pose2d... poses) {
        
    }

    /** @return Field2d widget. */
    public static Field2d getBaseField() {
        return field;
    }

    @Override
    public void periodic() {
        field.setRobotPose(odometer.getOdometryPoseMeters());
    }
}
