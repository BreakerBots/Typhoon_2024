// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.color;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** REV Color Sensor V3 implementing the Breaker device interface. */
public class BreakerColorSensorV3 extends BreakerGenericColorSensor {

  private ColorSensorV3 colorSensor;

  /**
   * Create a new BreakerColorSensor.
   * 
   * @param i2cPort I2C port for the color sensor.
   */
  public BreakerColorSensorV3(Port i2cPort) {
    colorSensor = new ColorSensorV3(i2cPort);
    deviceName = "Color_Sensor_V3";
  }

  @Override
  /** Current color detected by the sensor. */
  public Color getColor() {
    return colorSensor.getColor();
  }

  @Override
  /** Delivers RGB values plus IR value. */
  public int[] getRawColorsADC() {
    int[] colorVals = new int[4];
    colorVals[0] = colorSensor.getRawColor().red;
    colorVals[1] = colorSensor.getRawColor().green;
    colorVals[2] = colorSensor.getRawColor().blue;
    colorVals[3] = colorSensor.getRawColor().ir;
    return colorVals;
  }

  @Override
  /**
   * @return Sensor's proximity to its sensing target with 2047 being closest
   * and 0 being furthest.
   */
  public double getProximity() {
    return (double) (colorSensor.getProximity()) / 2048.0;
  }

  @Override
  public void runSelfTest() {
    faultStr = "";
    health = DeviceHealth.NOMINAL;
    if (!colorSensor.isConnected()) {
      health = DeviceHealth.INOPERABLE;
      faultStr = " COLOR_SENSOR_NOT_CONNECTED ";
    }
  }

}
