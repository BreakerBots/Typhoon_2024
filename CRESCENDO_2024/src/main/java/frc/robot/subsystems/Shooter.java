// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX piviot;
  private TalonFX flywheelTop, flywheelBottom;
  private CANcoder encoder;
  private BreakerBeamBreak beamBreak;
  public Shooter() {
    TalonFXConfiguration piviotMotorConfigs = new TalonFXConfiguration();

    piviotMotorConfigs.Slot0.kP = 0.0;
    piviotMotorConfigs.Slot0.kI = 0.0;
    piviotMotorConfigs.Slot0.kD = 0.0;
    piviotMotorConfigs.Slot0.kV = 0.0;
    piviotMotorConfigs.Slot0.kA = 0.0;
    piviotMotorConfigs.Slot0.kS = 0.0;
    piviotMotorConfigs.Slot0.kG = 0.0;
    piviotMotorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    piviotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    piviotMotorConfigs.MotionMagic.MotionMagicAcceleration = 0.0;
    piviotMotorConfigs.MotionMagic.MotionMagicJerk = 0.0;

    piviotMotorConfigs.Feedback.FeedbackRemoteSensorID = ;
    piviotMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    piviotMotorConfigs.HardwareLimitSwitch.

    
  }

  public static class 



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
