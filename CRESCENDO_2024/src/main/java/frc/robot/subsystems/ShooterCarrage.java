// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;

public class ShooterCarrage extends SubsystemBase {
  /** Creates a new PizzaBox. */
  private TalonFX pitchMotor;
  private WPI_TalonSRX hopperMotor;
  private Rotation2d targetPitch;
  private CarrageHopperState hopperState;
  private CANcoder pitchEnocder;
  private MotionMagicVoltage pitchMotionMagicRequest;
  private final BreakerBeamBreak beamBreak = new BreakerBeamBreak(0, true);
  public ShooterCarrage() {
    pitchEnocder = BreakerCANCoderFactory.createCANCoder(0, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, 0.0, SensorDirectionValue.Clockwise_Positive);

    TalonFXConfiguration pitchMotorConfig = new TalonFXConfiguration();
    pitchMotorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    pitchMotorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    pitchMotorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    pitchMotorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    pitchMotorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    pitchMotorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    pitchMotorConfig.Feedback.FeedbackRemoteSensorID = pitchEnocder.getDeviceID();
    pitchMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pitchMotorConfig.Feedback.RotorToSensorRatio = 0.0;

    pitchMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    pitchMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    pitchMotorConfig.MotionMagic.MotionMagicJerk = 0.0;

    pitchMotorConfig.Slot0.kP = 0.0;
    pitchMotorConfig.Slot0.kI = 0.0;
    pitchMotorConfig.Slot0.kD = 0.0;
    pitchMotorConfig.Slot0.kS = 0.0;
    pitchMotorConfig.Slot0.kA = 0.0;
    pitchMotorConfig.Slot0.kV = 0.0;
    pitchMotorConfig.Slot0.kG = 0.0;
    pitchMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pitchMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
  }


  


  public Rotation2d getPitch() {
    return new Rotation2d();
  }
  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public boolean isAtTargetState() {
    return true;
  }

  public void setTargetPitch(Rotation2d targetPitch) {
    
  }

  public void setHopperState(CarrageHopperState hopperState) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static enum CarrageHopperState {
    INTAKE,
    EXTAKE,
    NEUTRAL;
  }

  public static enum CarragePitchMode {
    STOW(Rotation2d.fromDegrees(0.0)),
    PASTA_ROLLER_HANDOFF(Rotation2d.fromDegrees(90.0)),
    INTAKE_FROM_HUMAN_PLAYER(Rotation2d.fromDegrees(55.0)),
    EJECT_NOTE(Rotation2d.fromDegrees(45.0)),
    HOLD_ARBITRARY,
    SHOOT_SPEAKER;

    private Optional<Rotation2d> targetAngleOptional;
    private CarragePitchMode(Optional<Rotation2d> targetAngleOptional) {
      this.targetAngleOptional = targetAngleOptional;
    }

    private CarragePitchMode(Rotation2d targetAngle) {
      this(Optional.of(targetAngle));
    }

    private CarragePitchMode() {
      this(Optional.empty());
    }

    public Optional<Rotation2d> getTargetAngle() {
      return targetAngleOptional;
    }
  }
}
