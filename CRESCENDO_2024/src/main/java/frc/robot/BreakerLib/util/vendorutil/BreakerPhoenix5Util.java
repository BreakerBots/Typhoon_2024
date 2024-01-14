// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.vendorutil;

import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Util class for CTRE devices */
public class BreakerPhoenix5Util {

  /**
   * Sets brake mode for given CTRE motors.
   * 
   * @param isEnabled True for brake mode, false for coast mode.
   * @param motors    CTRE motors.
   */
  public static void setBrakeMode(boolean isEnabled, BaseMotorController... motors) {
    for (BaseMotorController motor : motors) {
      motor.setNeutralMode((isEnabled ? NeutralMode.Brake : NeutralMode.Coast));
    }
  }

  public static double radiansToCANCoderNativeUnits(double radians) {
    return (radians / (2 * Math.PI)) * 4096.0;
  }

  /**
   * Logs an error to BreakerLog if designated error is discovered.
   * 
   * @param error   CTRE error code to detect.
   * @param message Message to log when the error is detected.
   */
  public static void checkError(ErrorCode error, String message) {
    if (error != ErrorCode.OK) {
      BreakerLog.logError(error + " - " + message);
    }
  }

  /**
   * Returns motor faults as a String.
   * 
   * @param motorFaults Faults from a CTRE motor controller.
   * @return All motor fault messages separated by spaces in a string.
   */
  public static String getMotorFaultsAsString(Faults motorFaults) {
    HashMap<Integer, String> map = new HashMap<Integer, String>();
    map.put(0, " device_under_6.5v ");
    map.put(1, " device_limit_switch_hit ");
    map.put(2, " device_limit_switch_hit ");
    map.put(3, " device_limit_switch_hit ");
    map.put(4, " device_limit_switch_hit ");
    map.put(5, " hardware_failure ");
    map.put(6, " device_activated_or_reset_while_robot_on ");
    map.put(9, " device_activated_or_reset_while_robot_on ");
    map.put(7, " sensor_overflow ");
    map.put(8, " sensor_out_of_phase ");
    map.put(10, " remote_sensor_not_detected ");
    map.put(11, " API_or_firmware_error ");
    map.put(12, " supply_voltage_above_rated_max ");
    map.put(13, " unstable_supply_voltage ");
    return BreakerVendorUtil.getDeviceFaultsAsString(motorFaults.toBitfield(), map);
  }

  /**
   * Get CTRE motor controller faults and device health.
   * 
   * @param motorFaults Motor controller faults.
   * @return Motor controller device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getMotorHealthAndFaults(Faults motorFaults) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_under_6.5v "));
    // map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    // map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    // map.put(3, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    // map.put(4, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    map.put(5, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " hardware_failure "));
    map.put(6, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_activated_or_reset_while_robot_on "));
    map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_activated_or_reset_while_robot_on "));
    map.put(7, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_overflow "));
    map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_out_of_phase "));
    map.put(10, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " remote_sensor_not_detected "));
    map.put(11, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " API_or_firmware_error "));
    map.put(12, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " supply_voltage_above_rated_max "));
    map.put(13, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " unstable_supply_voltage "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(motorFaults.toBitfield(), map);
  }

  /**
   * @param motor
   * @return Pair<DeviceHealth, String>
   */
  public static Pair<DeviceHealth, String> checkMotorFaultsAndConnection(IMotorController motor) {
    Faults motorFaults = new Faults();
    motor.getFaults(motorFaults);
    Pair<DeviceHealth, String> pair = getMotorHealthAndFaults(motorFaults);
    String retStr = pair.getSecond();
    DeviceHealth retHealth = pair.getFirst();
    if (motor.getFirmwareVersion() == -1) {
      retStr += " device_disconnected ";
      retHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth, String>(retHealth, retStr);
  }
  /**
   * Get CANdle faults and device health.
   * 
   * @param faults CANCoder faults.
   * @return CANCoder device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getCANdleHealthAndFaults(CANdleFaults faults) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " short_circut "));
    map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " thermal_fault "));
    map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " software_fuse "));
    map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " API_error "));
    map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " hardware_failure "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(faults.toBitfield(), map);
  }

  /**
   * Gets CANdle faults as a String.
   * 
   * @param faults CANdle faults.
   * @return All faults found as string. If no faults are found, "none" is
   *         returned.
   */
  public static String getCANdleFaultsAsString(CANdleFaults faults) {
    HashMap<Integer, String> map = new HashMap<Integer, String>();
    map.put(0, " short_circut ");
    map.put(1, " thermal_fault ");
    map.put(2, " software_fuse ");
    map.put(8, " API_error ");
    map.put(9, " hardware_failure ");
    return BreakerVendorUtil.getDeviceFaultsAsString(faults.toBitfield(), map);
  }
}
