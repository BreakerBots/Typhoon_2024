// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.color;


import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;

/** Add your docs here. */
public abstract class BreakerGenericColorSensor extends BreakerGenericDevice {

  /** Current color detected by the sensor. */
  public abstract Color getColor();

  /** Compare target color to detected color. */
  public boolean compareColor(Color comparisionColor) {
    return (comparisionColor.equals(getColor()));
  }

  /** Delivers RGB values plus IR/other values if supported */
  public abstract int[] getRawColorsADC();

  /**
   * @return Sensor's proximity to its sensing target with 1.0 being closest
   * and 0.0 being furthest.
   */
  public abstract double getProximity();

}
