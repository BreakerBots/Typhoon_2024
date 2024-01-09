// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Matter {
    private Translation3d robotRelativePosition;
    private double massKg;
    public Matter(Translation3d robotRelativePosition, double massKg) {
        this.robotRelativePosition = robotRelativePosition;
        this.massKg = massKg;
    }

    public void setMassKg(double massKg) {
        this.massKg = massKg;
    }

    public double getMassKg() {
        return massKg;
    }

    public void setRobotRelativePosition(Translation3d robotRelativePosition) {
        this.robotRelativePosition = robotRelativePosition;
    }

    public Translation3d getRobotRelativePosition() {
        return robotRelativePosition;
    }
    /**
   * Get the center mass of the object.
   *
   * @return center mass = position * mass
   */
  public Translation3d getMassMoment()
  {
    return robotRelativePosition.times(massKg);
  }
}
