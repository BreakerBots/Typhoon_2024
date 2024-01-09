// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.accelerometer;

import frc.robot.BreakerLib.physics.vector.BreakerVector3;

/** FRC 3-axis accelerometer interface. */
public interface BreakerGenericAccelerometer {

  /**
   * @return Array of raw accelerometer values in Gs.
   * <p>
   * x = 0, y = 1, z = 2.
   */
  public BreakerVector3 getRawAccelerometerVals();

  /** @return Unbiased accelerometer x-value in Gs. */
  public double getRawAccelX();

  /** @return Unbiased accelerometer y-value in Gs. */
  public double getRawAccelY();

  /** @return Unbiased accelerometer z-value in Gs. */
  public double getRawAccelZ();
}
