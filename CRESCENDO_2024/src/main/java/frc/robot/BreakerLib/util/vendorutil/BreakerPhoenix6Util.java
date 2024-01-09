// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.vendorutil;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerBaseStatusValue;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerPhoenix6Util {

    /**
   * Logs an error to BreakerLog if designated error is discovered.
   * 
   * @param statusCode  CTRE error code to detect.
   * @param message Message to log when the error is detected.
   */
  public static void checkStatusCode(StatusCode statusCode, String message) {
    if (statusCode != StatusCode.OK) {
      BreakerLog.logError(statusCode + " - " + message);
    }
  }


  public static void setBrakeMode(boolean isEnabled, TalonFX... motors) {
    for (TalonFX motor: motors) {
      setBrakeMode(motor, isEnabled);
    }
  }
  public static void setBrakeMode(TalonFX motor, boolean isEnabled) {
    motor.setNeutralMode(isEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public static Pair<DeviceHealth, String> checkPigeon2FaultsAndConnection(Pigeon2 pigeon2) {
    Pair<DeviceHealth, String> pair = getPigeon2HealthAndFaults(pigeon2);
    String retStr = pair.getSecond();
    DeviceHealth retHealth = pair.getFirst();
    if (!pigeon2.getVersion().getStatus().isOK()) {
      retStr += " device_disconnected ";
      retHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth, String>(retHealth, retStr);
  }

  
  public static Pair<DeviceHealth, String> getPigeon2HealthAndFaults(Pigeon2 pigeon2) {
    FaultCase[] faultCases = new FaultCase[] {
      new FaultCase(pigeon2.getFault_Hardware().getValue() || pigeon2.getStickyFault_Hardware().getValue(), DeviceHealth.INOPERABLE, " hardware_failure "),
      new FaultCase(pigeon2.getFault_BootupGyroscope().getValue(),  DeviceHealth.INOPERABLE, " gyroscope_bootup_check_failed "),
      new FaultCase(pigeon2.getFault_BootupAccelerometer().getValue(),  DeviceHealth.INOPERABLE, " accelerometer_bootup_check_failed "),
      new FaultCase(pigeon2.getFault_BootupMagnetometer().getValue(),  DeviceHealth.INOPERABLE, " magnetometer_bootup_check_failed "),
      new FaultCase(pigeon2.getFault_Undervoltage().getValue(),  DeviceHealth.FAULT, " device_supply_voltage_below_6.5v "),
      new FaultCase(pigeon2.getFault_DataAcquiredLate().getValue(),  DeviceHealth.FAULT, " motion_stack_data_acquisition_slower_than_expected "),
      new FaultCase(pigeon2.getFault_LoopTimeSlow().getValue(),  DeviceHealth.FAULT, " motion_stack_loop_time_slower_than_expected "),
      new FaultCase(pigeon2.getFault_SaturatedAccelometer().getValue(),  DeviceHealth.FAULT, " accelometer_values_saturated "),
      new FaultCase(pigeon2.getFault_SaturatedMagnetometer().getValue(), DeviceHealth.FAULT, " magnetometer_values_satureted "),
      new FaultCase(pigeon2.getFault_SaturatedGyroscope().getValue(), DeviceHealth.FAULT, " gyroscope_values_saturated"),
      new FaultCase(pigeon2.getFault_BootDuringEnable().getValue(),  DeviceHealth.FAULT, " device_boot_or_reset_while_robot_enabled "),
      new FaultCase(pigeon2.getFault_BootIntoMotion().getValue(), DeviceHealth.FAULT, " device_boot_while_in_motion "),
      new FaultCase(pigeon2.getFault_UnlicensedFeatureInUse().getValue(), DeviceHealth.FAULT, " unlicensed_feature_in_use ")
    };

    return getDeviceHealthAndFaults(faultCases);
  }


  /**
   * @param motor
   * @return Pair<DeviceHealth, String>
   */
  public static Pair<DeviceHealth, String> checkMotorFaultsAndConnection(CoreTalonFX motor) {
    Pair<DeviceHealth, String> pair = getMotorHealthAndFaults(motor);
    String retStr = pair.getSecond();
    DeviceHealth retHealth = pair.getFirst();
    if (!motor.getVersion().getStatus().isOK()) {
      retStr += " device_disconnected ";
      retHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth, String>(retHealth, retStr);
  }

  public static Pair<DeviceHealth, String> getMotorHealthAndFaults(CoreTalonFX motor) {
    FaultCase[] faultCases = new FaultCase[] {
      new FaultCase(motor.getFault_Hardware().getValue() || motor.getStickyFault_Hardware().getValue(), DeviceHealth.INOPERABLE, " hardware_failure "),
      new FaultCase(motor.getFault_DeviceTemp().getValue(), DeviceHealth.INOPERABLE, " device_temperature_exceeded_limit "),
      new FaultCase(motor.getFault_ProcTemp().getValue(), DeviceHealth.INOPERABLE, " processor_temperature_exceeded_limit "),
      new FaultCase(motor.getFault_Undervoltage().getValue(), DeviceHealth.FAULT, " device_under_6.5v "),
      new FaultCase(motor.getFault_OverSupplyV().getValue(), DeviceHealth.FAULT, " supply_voltage_above_rated_max "),
      new FaultCase(motor.getFault_UnstableSupplyV().getValue(), DeviceHealth.FAULT, " unstable_supply_voltage "),
      new FaultCase(motor.getFault_BootDuringEnable().getValue(), DeviceHealth.FAULT, " device_boot_or_reset_while_robot_enabled "),
      new FaultCase(motor.getFault_FusedSensorOutOfSync().getValue(), DeviceHealth.FAULT, " fused_sensor_out_of_sync "),
      new FaultCase(motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue(), DeviceHealth.FAULT, " using_fused_CANcoder_feature_while_unlicensed "),
      new FaultCase(motor.getFault_RemoteSensorDataInvalid().getValue(), DeviceHealth.FAULT, " remote_sensor_data_invalid "),
      new FaultCase(motor.getFault_RemoteSensorPosOverflow().getValue(), DeviceHealth.FAULT, " remote_sensor_pos_overflow "),
      new FaultCase(motor.getFault_RemoteSensorReset().getValue(), DeviceHealth.FAULT, " remote_sensor_reset "),
      new FaultCase(motor.getFault_BridgeBrownout().getValue(), DeviceHealth.FAULT, " bridge_brownout "),
      new FaultCase(motor.getFault_MissingDifferentialFX().getValue(), DeviceHealth.FAULT, " differential_control_TalonFX_not_detected "),
      new FaultCase(motor.getFault_UnlicensedFeatureInUse().getValue(), DeviceHealth.FAULT, " unlicensed_feature_in_use "),
    };

    return getDeviceHealthAndFaults(faultCases);
  }

  public static Pair<DeviceHealth, String> getCANcoderHealthAndFaults(CANcoder canCoder) {

    FaultCase[] faultCases = new FaultCase[] {
      new FaultCase(canCoder.getFault_Hardware().getValue() || canCoder.getStickyFault_Hardware().getValue(), DeviceHealth.INOPERABLE, " hardware_failure "),
      new FaultCase(canCoder.getFault_BadMagnet().getValue(), DeviceHealth.INOPERABLE, " magnet_too_weak "),
      new FaultCase(canCoder.getFault_Undervoltage().getValue(), DeviceHealth.FAULT, " device_under_6.5v "),
      new FaultCase(canCoder.getFault_BootDuringEnable().getValue(), DeviceHealth.FAULT, " device_boot_or_reset_while_robot_enabled "),
      new FaultCase(canCoder.getFault_UnlicensedFeatureInUse().getValue(), DeviceHealth.FAULT, " unlicensed_feature_in_use "),
    };

    return getDeviceHealthAndFaults(faultCases);
  }

  /**
   * @param canCoder
   * @return Pair<DeviceHealth, String>
   */
  public static Pair<DeviceHealth, String> checkCANcoderFaultsAndConnection(CANcoder canCoder) {
    Pair<DeviceHealth, String> pair = getCANcoderHealthAndFaults(canCoder);
    String retStr = pair.getSecond();
    DeviceHealth retHealth = pair.getFirst();
    if (!canCoder.getVersion().getStatus().isOK()) {
      retStr += " device_disconnected ";
      retHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth, String>(retHealth, retStr);
  }

  private static Pair<DeviceHealth, String> getDeviceHealthAndFaults(FaultCase... faultCases) {
    StringBuilder work = new StringBuilder();
    DeviceHealth health = DeviceHealth.NOMINAL;
    for (FaultCase faultCase: faultCases) {
      if (faultCase.flag) {
        work.append(faultCase.messsage);
        if (health.ordinal() < faultCase.effect.ordinal()) {
          health = faultCase.effect;
        }
      }
    }
    return new Pair<DeviceHealth,String>(health, work.toString());
  }


  private static class FaultCase {
    public boolean flag;
    public DeviceHealth effect;
    public String messsage;
    public FaultCase(boolean flag, DeviceHealth effect, String messsage) {
      this.flag = flag;
      this.effect = effect;
      this.messsage = messsage;
    }
  }

  public static class TalonFXRemoteLimitController extends SubsystemBase {
    private BooleanSupplier limitTriggeredSupplier;
    private LimitDirection limitDirection;
    private double limitTriggeredRotorPosition;
    private TalonFX motor;
    private boolean prevState, limitEnabled;
    public TalonFXRemoteLimitController(TalonFX motor, BooleanSupplier limitTriggeredSupplier, LimitDirection limitDirection, double limitTriggeredRotorPosition) {
        configMotor(motor, limitDirection, limitTriggeredRotorPosition, false);
        prevState = false;
        limitEnabled = false;
    }

    private static void configMotor(TalonFX motor, LimitDirection limitDirection, double limitTriggeredRotorPosition, boolean enableLimit) {
        SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(softLimitConfig);
        if (limitDirection == LimitDirection.FORWARD) {
            softLimitConfig.ForwardSoftLimitThreshold = limitTriggeredRotorPosition;
            softLimitConfig.ForwardSoftLimitEnable = enableLimit;
        } else {
            softLimitConfig.ReverseSoftLimitThreshold = limitTriggeredRotorPosition;
            softLimitConfig.ReverseSoftLimitEnable = enableLimit;
        }
    }

    public boolean isLimitTriggered() {
        return limitTriggeredSupplier.getAsBoolean();
    }

    @Override
    public void periodic() {
        boolean curState = isLimitTriggered();
        if (curState && !prevState) {
            if (!limitEnabled) {
                configMotor(motor, limitDirection, limitTriggeredRotorPosition, true);
            }
            motor.setPosition(limitTriggeredRotorPosition);
        }
        prevState = curState;
    }

    public static enum LimitDirection {
        FORWARD,
        REVERSE
    }
}

  
}
