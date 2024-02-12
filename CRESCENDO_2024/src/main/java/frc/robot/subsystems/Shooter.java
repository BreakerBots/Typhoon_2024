// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.ShooterTarget.FireingSolution;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX piviotMotor;
  private TalonFX flywheelLeft, flywheelRight;
  private WPI_TalonSRX hopper;
  private ShooterTarget target;
  private ShooterState state;
  private VelocityVoltage flywheelVelRequest;
  private Follower flywheelFollowRequest;
  private MotionMagicVoltage pivotMotionMagicRequest;
  private FireingSolution latestFireingSolution;
  public Shooter(ShooterTarget defaultTarget) {
    target = defaultTarget;
  }

  private void configFlywheel() {

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
    return true;
  }

  public boolean isAtGoal() {

  }

  public boolean isAtAngleGoal() {

  }

  public boolean isAtFlywheelGoal() {

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

  }

  public Rotation2d getShooterAngle() {

  }

  private void pushControlRequests(double hopperDutyCycle, double piviotPos, double flywheelVel) {
    hopper.set(hopperDutyCycle);
    piviotMotor.setControl(pivotMotionMagicRequest.withPosition(piviotPos));
    flywheelLeft.setControl(flywheelVelRequest.withVelocity(flywheelVel));
    flywheelRight.setControl(flywheelFollowRequest);
  }
  
  @Override
  public void periodic() {
    latestFireingSolution = target.getFireingSolution();
    switch (state) {
      case SHOOT_TO_TARGET:
      case TRACK_TARGET:
        pushControlRequests(state.getHopperState().getDutyCycle(), latestFireingSolution.fireingVec().getVectorRotation().getRotations(), latestFireingSolution.fireingVec().getMagnitude());
        break;
      case STOW:
      case INTAKE_TO_SHOOTER_HANDOFF:
      case SHOOTER_TO_INTAKE_HANDOFF:
      default:
        pushControlRequests(state.getHopperState().getDutyCycle(), STOW_ANGLE.getRotations(), latestFireingSolution.fireingVec().getMagnitude());
        break;

    }
  }
}
