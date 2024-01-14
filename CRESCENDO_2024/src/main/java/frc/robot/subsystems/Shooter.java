// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX piviot;
  private TalonFX flywheelTop, flywheelBottom;
  private CANcoder encoder;
  private BreakerBeamBreak beamBreak;


  private VelocityVoltage flywheelVelocityControl;
  private MotionMagicVoltage shooterPitchControl;

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

  public static ShooterState getTargetState() {

  }

  public static ShooterState getCurrentState() {
    
  }

  public static boolean isAtTargetState() {

  }

  public static Command SetShooterStateCommand(ShooterState targetShooterState, boolean waitForSuccess) {
    return 
  }

  public static void SetShooterState(ShooterState targetShooterState) {

  }

  public static class ShooterState {
    public static final ShooterState 
    public ShooterState(FeederState feederState, Rotation2d shooterAngle, double flywheelRPS) {

    }

    @Override
    public boolean equals(Object obj) {
        return super.equals(obj);
    }
  }

  public static enum FeederState {
    PULL_FROM_INTAKE(0.0),
    PUSH_TO_INTAKE(0.0),
    PULL_FROM_AMP_MECH(0.0),
    PUSH_TO_AMP_MECH(0.0),
    FEED_FLYWHEEL(0.0),
    NEUTRAL(0.0);
    private double dutyCycle;
    private FeederState(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
