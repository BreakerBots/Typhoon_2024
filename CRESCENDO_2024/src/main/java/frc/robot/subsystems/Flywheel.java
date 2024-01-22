// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;
import static frc.robot.Constants.FlywheelConstants.*;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private TalonFX flywheelA, flywheelB;
  private double targetVelRPS;
  private FlywheelPresisionType requiredPresision;
  public Flywheel() {
    
  } 

  public boolean isAtTargetVelocity() {
    if (requiredPresision != FlywheelPresisionType.NONE) {
      double velTol = FINE_VELOCITY_TOLERENCE;
      double accelTol = FINE_ACCELERATION_TOLERENCE;
      if (requiredPresision == FlywheelPresisionType.COARSE) {
        velTol = COARSE_VELOCITY_TOLERENCE;
        accelTol = COARSE_ACCELERATION_TOLERENCE;
      }
      double vel = flywheelA.getVelocity().getValue();
      double accel = flywheelA.getAcceleration().getValue(); 
      return MathUtil.isNear(targetVelRPS, vel, velTol) && MathUtil.isNear(0.0, accel, accelTol);
    }
    return true;
  }

  public double getVelocity() {
    return flywheelA.getVelocity().getValue();
  }

  public double getTangetialVelocity() {
    return getVelocity() * METERS_PER_SEC_PER_MOTOR_ROT;
  }

  public void setTargetState(double targetVelRPS, FlywheelPresisionType requiredPresision) {
    this.targetVelRPS = targetVelRPS;
    this.requiredPresision = requiredPresision;
    
  }

  public static enum FlywheelPresisionType {
    COARSE,
    FINE,
    NONE
  }

  public static enum FlywheelState {
    NEUTRAL(Optional.of(0.0), FlywheelPresisionType.NONE),
    INTAKE(Optional.of(MAX_SPEED * 0.2), FlywheelPresisionType.COARSE),
    PASTA_ROLLER_HANDOFF(Optional.of(MAX_SPEED * 0.3), FlywheelPresisionType.COARSE),
    EJECT_NOTE(Optional.of(MAX_SPEED * 0.6), FlywheelPresisionType.COARSE),
    SHOOT_SPEAKER(Optional.empty(), FlywheelPresisionType.FINE),
    IDLE(Optional.empty(), FlywheelPresisionType.COARSE);
    private Optional<Double> requiredVelocityRPS;
    private FlywheelPresisionType requiredPresision;
    private FlywheelState(Optional<Double> requiredVelocityRPS, FlywheelPresisionType requiredPresision) {
      this.requiredPresision = requiredPresision;
      this.requiredVelocityRPS = requiredVelocityRPS;
    }

    public FlywheelPresisionType getRequiredPresision() {
        return requiredPresision;
    }

    public Optional<Double> getRequiredVelocityRPS() {
        return requiredVelocityRPS;
    }
  }

  @Override
  public void periodic() {
  }
}
