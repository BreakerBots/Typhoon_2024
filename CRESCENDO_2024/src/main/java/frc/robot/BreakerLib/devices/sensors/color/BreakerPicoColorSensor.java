// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.color;

import java.util.Objects;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.devices.sensors.color.BreakerPicoColorSensorLowLevel.RawColor;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** REV Color Sensor V3 implementing the Breaker device interface. */
public class BreakerPicoColorSensor {

  private BreakerPicoColorSensorLowLevel pico;
  private BreakerPicoColorSensorInstance sensor0, sensor1;

  /**
   * Create a new BreakerColorSensor.
   */
  public BreakerPicoColorSensor() {
    pico = new BreakerPicoColorSensorLowLevel();
  }

  public BreakerPicoColorSensorInstance getSensor0() {
    if (Objects.isNull(sensor0)) {
      sensor0 = new BreakerPicoColorSensorInstance(pico, true);
    }
    return sensor0;
  }

  public BreakerPicoColorSensorInstance getSensor1() {
    if (Objects.isNull(sensor1)) {
      sensor1 = new BreakerPicoColorSensorInstance(pico, false);
    }
    return sensor1;
  }

  public static class BreakerPicoColorSensorInstance extends BreakerGenericColorSensor {
    private BreakerPicoColorSensorLowLevel pico;
    private boolean isSensor0;
    private BreakerPicoColorSensorInstance(BreakerPicoColorSensorLowLevel pico, boolean isSensor0) {
      this.pico = pico;
      this.isSensor0 = isSensor0;
      deviceName = isSensor0 ? "Pico_Color_Sensor_0" : "Pico_Color_Sensor_1";
    }

    public RawColor getRawColor() {
      return isSensor0 ? pico.getRawColor0() : pico.getRawColor1();
    }

    @Override
    /** Current color detected by the sensor. */
    public Color getColor() {
      RawColor rc = getRawColor();
      double r = (double) rc.red;
      double g = (double) rc.green;
      double b = (double) rc.blue;
      double mag = r + g + b;
      return new Color(r / mag, g / mag, b / mag);
    }

    @Override
    /** Delivers RGB values plus IR value. */
    public int[] getRawColorsADC() {
      int[] colorVals = new int[4];
      RawColor rc = getRawColor();
      colorVals[0] =  rc.red;
      colorVals[1] =  rc.green;
      colorVals[2] =  rc.blue;
      colorVals[3] =  rc.ir;
      return colorVals;
    }

    @Override
    public double getProximity() {
      return (double) (isSensor0 ? pico.getProximity0() : pico.getProximity1()) / 2048.0;
    }

    @Override
    public void runSelfTest() {
      faultStr = "";
      health = DeviceHealth.NOMINAL;
      if (!(isSensor0 ? pico.isSensor0Connected() : pico.isSensor1Connected())) {
        health = DeviceHealth.INOPERABLE;
        faultStr = isSensor0 ? " COLOR_SENSOR_0_NOT_CONNECTED " : " COLOR_SENSOR_1_NOT_CONNECTED ";
      }
    }
  }

}
