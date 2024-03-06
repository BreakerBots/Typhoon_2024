// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import static frc.robot.Constants.ShooterConstants.SHOOTER_IDLE;
import static frc.robot.Constants.ShooterConstants.SHOOTER_PIVOT_ID;
import static frc.robot.Constants.ShooterConstants.STOW_ANGLE;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX piviotMotor;
  private TalonFX flywheelLeft, flywheelRight;
  private CANcoder pivotEncoder;
  private WPI_TalonSRX hopper;
  private Supplier<FireingSolution> target;
  private ShooterState state;
  private Supplier<Double> flywheelVelSup;
  private Supplier<Double> flywheelAccelSup;
  private Supplier<Double> pivotPosSup;
  private Supplier<Double> pivotVelSup;
  private VelocityVoltage flywheelVelRequest;
  private Follower flywheelFollowRequest;
  private MotionMagicVoltage pivotMotionMagicRequest;
  private FireingSolution latestFireingSolution;
  private BreakerBeamBreak beamBreak;
  private CoastOut flywheelCoastRequest;
  
  public Shooter(Supplier<FireingSolution> defaultTarget) {
    target = defaultTarget;
    beamBreak = new BreakerBeamBreak(3, true);
    piviotMotor = new TalonFX(SHOOTER_PIVOT_ID);
    flywheelLeft = new TalonFX(LEFT_FLYWHEEL_ID);
    flywheelRight = new TalonFX(RIGHT_FLYWHEEL_ID);
    hopper = new WPI_TalonSRX(HOPPER_ID);
    hopper.setNeutralMode(NeutralMode.Brake);
    pivotEncoder = BreakerCANCoderFactory.createCANCoder(PIVOT_ENCODER_ID, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, PITCH_ENCODER_OFFSET, SensorDirectionValue.CounterClockwise_Positive);
    configPivot();
    configFlywheel();
    state = ShooterState.STOW;
    flywheelCoastRequest = new CoastOut();
  }

  private void configFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = 10;
    config.CurrentLimits.SupplyCurrentThreshold = 100;
    config.CurrentLimits.SupplyTimeThreshold = 3.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    flywheelRight.getConfigurator().apply(config);
    config.Slot0.kP = 0.005;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.01;
    config.Slot0.kV = 0.1168;
    config.Slot0.kS = 0.17;
    config.Slot0.kA = 0.1168;
    flywheelLeft.getConfigurator().apply(config);
    flywheelVelRequest = new VelocityVoltage(0.0);
    flywheelFollowRequest = new Follower(LEFT_FLYWHEEL_ID, true);
    flywheelVelSup = flywheelLeft.getVelocity().asSupplier();
    flywheelAccelSup = flywheelLeft.getAcceleration().asSupplier();
  }

  private void configPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackRemoteSensorID = PIVOT_ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = PITCH_RATIO;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.kP = PITCH_KP;
    config.Slot0.kI = PITCH_KI;
    config.Slot0.kD = PITCH_KD;
    config.Slot0.kV = PITCH_KV;
    config.Slot0.kS = PITCH_KS;
    config.Slot0.kA = PITCH_KA;
    config.Slot0.kG = PITCH_KG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = 4.0;
    config.MotionMagic.MotionMagicAcceleration = 0.6;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyTimeThreshold = 1.5;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PITCH_MAX_ROT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PITCH_MIN_ROT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false, false, false);
    piviotMotor.getConfigurator().apply(config);
    pivotPosSup = piviotMotor.getPosition().asSupplier();
    pivotVelSup = piviotMotor.getVelocity().asSupplier();
  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public ShooterState getState() {
    return state;
  }

  public void setActiveTarget(Supplier<FireingSolution> target) {
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
    return MathUtil.isNear(pivotMotionMagicRequest.Position, pivotPosSup.get(), 0.006) && MathUtil.isNear(0.0, pivotVelSup.get(),  0.01);//0.5, 0.1
  }

  public boolean isAtFlywheelGoal() {
    return MathUtil.isNear(flywheelVelSup.get(), flywheelVelSup.get(), 3.0) && MathUtil.isNear(0.0, flywheelAccelSup.get(), 0.5) ;//5.0, 0.5
  }


  public static enum ShooterHopperState {
    FORWARD(-0.7),
    REVERSE(0.4),
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
    TRACK_TARGET_IDLE(ShooterHopperState.NEUTRAL),
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
    return flywheelVelSup.get();
  }

  public double getFlywheelAccel() {
    return flywheelAccelSup.get();
  }

  public Rotation2d getShooterAngle() {
    return Rotation2d.fromRotations(pivotPosSup.get());
  }

  private void pushControlRequests(double hopperDutyCycle, double piviotPos, double flywheelVel) {
    hopper.set(hopperDutyCycle);
    piviotMotor.setControl(pivotMotionMagicRequest.withPosition(piviotPos));
    if (flywheelVel > 0.0) {
      flywheelLeft.setControl(flywheelVelRequest.withVelocity(flywheelVel));
    } else {
      flywheelLeft.setControl(flywheelCoastRequest);
    }
    flywheelRight.setControl(flywheelFollowRequest);
  }
  
  @Override
  public void periodic() {
    BreakerLog.recordOutput("Shooter Has Note", hasNote());
    if (RobotState.isDisabled()) {
      hopper.set(0.0);
    }
    // flywheelLeft.setControl(flywheelVelRequest.withVelocity(30));
    // flywheelRight.setControl(flywheelFollowRequest);
    latestFireingSolution = target.get();
    switch (state) {
      case SHOOT_TO_TARGET:
      case TRACK_TARGET:
      case TRACK_TARGET_IDLE:
        pushControlRequests(state.getHopperState().getDutyCycle(), latestFireingSolution.fireingVec().getVectorRotation().getRotations(), state == ShooterState.TRACK_TARGET_IDLE ? SHOOTER_IDLE : latestFireingSolution.fireingVec().getMagnitude());
        break;
      case STOW:
      case INTAKE_TO_SHOOTER_HANDOFF:
      case SHOOTER_TO_INTAKE_HANDOFF:
      default:
        pushControlRequests(state.getHopperState().getDutyCycle(), STOW_ANGLE.getRotations(), SHOOTER_IDLE);
        break;

    }
  }
}
