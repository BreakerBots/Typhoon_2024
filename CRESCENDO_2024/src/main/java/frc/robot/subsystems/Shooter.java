// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShooterConstants.HOPPER_ID;
import static frc.robot.Constants.ShooterConstants.LEFT_FLYWHEEL_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_ENCODER_OFFSET;
import static frc.robot.Constants.ShooterConstants.PITCH_KA;
import static frc.robot.Constants.ShooterConstants.PITCH_KD;
import static frc.robot.Constants.ShooterConstants.PITCH_KG;
import static frc.robot.Constants.ShooterConstants.PITCH_KI;
import static frc.robot.Constants.ShooterConstants.PITCH_KP;
import static frc.robot.Constants.ShooterConstants.PITCH_KS;
import static frc.robot.Constants.ShooterConstants.PITCH_KV;
import static frc.robot.Constants.ShooterConstants.PITCH_MAX_ROT;
import static frc.robot.Constants.ShooterConstants.PITCH_MIN_ROT;
import static frc.robot.Constants.ShooterConstants.PITCH_RATIO;
import static frc.robot.Constants.ShooterConstants.PIVOT_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.RIGHT_FLYWHEEL_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_PIVOT_ID;
import static frc.robot.Constants.ShooterConstants.STOW_ANGLE;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX piviotMotor;
  private TalonFX flywheelLeft, flywheelRight;
  private CANcoder pivotEncoder;
  private WPI_TalonSRX hopper;
  private ShooterTarget target;
  private ShooterState state;
  private DoubleSupplier flywheelVelSup;
  private DoubleSupplier flywheelAccelSup;
  private DoubleSupplier pivotPosSup;
  private DoubleSupplier pivotVelSup;
  private VelocityVoltage flywheelVelRequest;
  private Follower flywheelFollowRequest;
  private MotionMagicVoltage pivotMotionMagicRequest;
  private FireingSolution latestFireingSolution;
  private BreakerBeamBreak beamBreak;
  private BreakerXboxController con;
  public Shooter(/*ShooterTarget defaultTarget*/BreakerXboxController con) {
   // target = defaultTarget;
    this.con = con;
    beamBreak = new BreakerBeamBreak(0, true);
    piviotMotor = new TalonFX(SHOOTER_PIVOT_ID);
    flywheelLeft = new TalonFX(LEFT_FLYWHEEL_ID);
    flywheelRight = new TalonFX(RIGHT_FLYWHEEL_ID);
    hopper = new WPI_TalonSRX(HOPPER_ID);
    pivotEncoder = BreakerCANCoderFactory.createCANCoder(PIVOT_ENCODER_ID, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, PITCH_ENCODER_OFFSET, SensorDirectionValue.CounterClockwise_Positive);
    configPivot();
  }

  private void configFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = 10;
    config.CurrentLimits.SupplyCurrentThreshold = 60;
    config.CurrentLimits.SupplyTimeThreshold = 3.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelRight.getConfigurator().apply(config);
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    flywheelLeft.getConfigurator().apply(config);
    flywheelVelRequest = new VelocityVoltage(0.0);
    flywheelFollowRequest = new Follower(LEFT_FLYWHEEL_ID, true);
  }

  private void configPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackRemoteSensorID = PIVOT_ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = PITCH_RATIO;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.kP = PITCH_KP;
    config.Slot0.kI = PITCH_KI;
    config.Slot0.kD = PITCH_KD;
    config.Slot0.kV = PITCH_KV;
    config.Slot0.kS = PITCH_KS;
    config.Slot0.kA = PITCH_KA;
    config.Slot0.kG = PITCH_KG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.MotionMagic.MotionMagicCruiseVelocity = 0.1;
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyTimeThreshold = 1.5;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PITCH_MIN_ROT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PITCH_MAX_ROT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    piviotMotor.getConfigurator().apply(config);
  }

  public void setState(ShooterState state) {

  }

  public void setActiveTarget(ShooterTarget target) {
    this.target = target;
  }

  public FireingSolution getActiveTargetFireingSolution() {
      return latestFireingSolution;
  }

  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public boolean isAtGoal() {
    return isAtAngleGoal() && isAtFlywheelGoal();
  }

  public boolean isAtAngleGoal() {
    return MathUtil.isNear(pivotMotionMagicRequest.Position, pivotPosSup.getAsDouble(), Units.degreesToRotations(0.5)) && MathUtil.isNear(0.0, pivotVelSup.getAsDouble(),  Units.degreesToRotations(0.1));
  }

  public boolean isAtFlywheelGoal() {
    return MathUtil.isNear(pivotMotionMagicRequest.Position, flywheelVelSup.getAsDouble(), 5.0 / 60.0) && MathUtil.isNear(0.0, flywheelAccelSup.getAsDouble(), 0.5 / 60) ;
  }


  public static enum ShooterHopperState {
    FORWARD(0.75),
    REVERSE(-0.4),
    NEUTRAL(0.0);
    private double dutyCycle;
    private ShooterHopperState(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }

    public double getDutyCycle() {
        return dutyCycle;
    }
  }

  public static enum ShooterState {
    TRACK_TARGET(ShooterHopperState.NEUTRAL),
    SHOOT_TO_TARGET(ShooterHopperState.FORWARD),
    INTAKE_TO_SHOOTER_HANDOFF(ShooterHopperState.FORWARD),
    SHOOTER_TO_INTAKE_HANDOFF(ShooterHopperState.REVERSE),
    STOW(ShooterHopperState.NEUTRAL);

    private ShooterHopperState hopperState;
    private ShooterState(ShooterHopperState hopperState) {
      this.hopperState = hopperState;
    }

    public ShooterHopperState getHopperState() {
        return hopperState;
    }
  }

  public double getFlywheelVel() {
    return flywheelVelSup.getAsDouble();
  }

  public double getFlywheelAccel() {
    return flywheelAccelSup.getAsDouble();
  }

  public Rotation2d getShooterAngle() {
    return Rotation2d.fromRotations(pivotPosSup.getAsDouble());
  }

  public void setFlywheelSpeed(double speed) {
    flywheelLeft.set(-speed);
    flywheelRight.set(speed);
  }

  public void setHopSpeed(double speed) {
    hopper.set(speed);
  }

  private void pushControlRequests(double hopperDutyCycle, double piviotPos, double flywheelVel) {
    hopper.set(hopperDutyCycle);
    piviotMotor.setControl(pivotMotionMagicRequest.withPosition(piviotPos));
    flywheelLeft.setControl(flywheelVelRequest.withVelocity(flywheelVel));
    flywheelRight.setControl(flywheelFollowRequest);
  }
  
  @Override
  public void periodic() {
    Logger.recordOutput("Shooter Has Note", hasNote());
    if (RobotState.isDisabled()) {
      hopper.set(0.0);
      flywheelLeft.set(0.0);
      flywheelRight.set(0.0);
    }
    // latestFireingSolution = target.getFireingSolution();
    // switch (state) {
    //   case SHOOT_TO_TARGET:
    //   case TRACK_TARGET:
    //     pushControlRequests(state.getHopperState().getDutyCycle(), latestFireingSolution.fireingVec().getVectorRotation().getRotations(), latestFireingSolution.fireingVec().getMagnitude());
    //     break;
    //   case STOW:
    //   case INTAKE_TO_SHOOTER_HANDOFF:
    //   case SHOOTER_TO_INTAKE_HANDOFF:
    //   default:
    //     pushControlRequests(state.getHopperState().getDutyCycle(), STOW_ANGLE.getRotations(), latestFireingSolution.fireingVec().getMagnitude());
    //     break;

    // }
  }
}
