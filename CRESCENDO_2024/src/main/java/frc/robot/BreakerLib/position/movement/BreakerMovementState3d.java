// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.movement;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.physics.Breaker6AxisForces;

/** Represents an objects 3 dimentional (linear: XYZ / Angular: YPR) 0th through nth derivitives at any given time */
public class BreakerMovementState3d {
    private Pose3d position;
    private Breaker6AxisForces[] derivitivesOfPosition;

    /**
     * Creates a new BreakerMovementState2d using a position and its 1st through nth derivitives.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param jerk
     */
    public BreakerMovementState3d(Pose3d position, Breaker6AxisForces... derivitivesOfPosition) {
        this.position = position;
        this.derivitivesOfPosition = derivitivesOfPosition;
    }

     /**
     * Creates a new BreakerMovementState2d and uses default values for the position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState3d() {
        position = new Pose3d();
        derivitivesOfPosition = new Breaker6AxisForces[0];
    }

    public Pose3d estimateFuturePose(double deltaTimeSeconds) {
        double prevDirX = derivitivesOfPosition[derivitivesOfPosition.length - 1].getLinearForces().getX();
        double prevDirY = derivitivesOfPosition[derivitivesOfPosition.length - 1].getLinearForces().getY();
        double prevDirZ = derivitivesOfPosition[derivitivesOfPosition.length - 1].getLinearForces().getZ();
        double prevDirYaw = derivitivesOfPosition[derivitivesOfPosition.length - 1].getAngularForces().getMagnitudeYaw();
        double prevDirPitch = derivitivesOfPosition[derivitivesOfPosition.length - 1].getAngularForces().getMagnitudePitch();
        double prevDirRoll = derivitivesOfPosition[derivitivesOfPosition.length - 1].getAngularForces().getMagnitudeRoll();
        for (int i = derivitivesOfPosition.length - 1; i >= 0; i++) {
            prevDirX = derivitivesOfPosition[i].getLinearForces().getX() + (prevDirX * deltaTimeSeconds);
            prevDirY = derivitivesOfPosition[i].getLinearForces().getY() + (prevDirY * deltaTimeSeconds);
            prevDirZ = derivitivesOfPosition[i].getLinearForces().getZ() + (prevDirZ * deltaTimeSeconds);
            prevDirYaw = derivitivesOfPosition[i].getAngularForces().getMagnitudeYaw() + (prevDirYaw * deltaTimeSeconds);
            prevDirPitch = derivitivesOfPosition[i].getAngularForces().getMagnitudePitch() + (prevDirPitch * deltaTimeSeconds);
            prevDirRoll = derivitivesOfPosition[i].getAngularForces().getMagnitudeRoll() + (prevDirRoll * deltaTimeSeconds);
        }
        double x = position.getX() + (prevDirX * deltaTimeSeconds);
        double y = position.getY() + (prevDirY * deltaTimeSeconds);
        double z = position.getZ() + (prevDirZ * deltaTimeSeconds);
        double yaw = position.getRotation().getZ() + (prevDirYaw * deltaTimeSeconds);
        double pitch = position.getRotation().getY() + (prevDirPitch * deltaTimeSeconds);
        double roll = position.getRotation().getZ() + (prevDirRoll * deltaTimeSeconds);
        return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    }

    /**
     * @return The position component of this BreakerMovementState2d
     */
    public Pose3d getPositionComponent() {
        return position;
    }
 
    public Breaker6AxisForces[] getDerivitivesOfPosition() {
        return Arrays.copyOf(derivitivesOfPosition, derivitivesOfPosition.length);
    }


    /** follows indexes of dirivitive array, so 1st dirivitive would be at index 0 */
    public Breaker6AxisForces getDirivitiveFromIndex(int indexOfDirivitve) {
        if (indexOfDirivitve >= derivitivesOfPosition.length || indexOfDirivitve < 0 || derivitivesOfPosition[indexOfDirivitve] == null) {
            return new Breaker6AxisForces();
        }
        return derivitivesOfPosition[indexOfDirivitve];
    }

    @Override
    public String toString() {
        return String.format("Breaker3AxisForces(Position: %s, Dirivitives: %s)", position, Arrays.toString(derivitivesOfPosition));
    }
}
