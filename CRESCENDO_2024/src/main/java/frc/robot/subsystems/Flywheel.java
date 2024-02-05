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
  public Flywheel() {
    
  } 

  public boolean isAtTargetVelocity() {
    double vel = flywheelA.getVelocity().getValue();
    double accel = flywheelA.getAcceleration().getValue(); 
    return MathUtil.isNear(targetVelRPS, vel, VELOCITY_TOLERENCE) && MathUtil.isNear(0.0, accel, ACCELERATION_TOLERENCE);
  }

  public double getVelocity() {
    return flywheelA.getVelocity().getValue();
  }

  public double getTangetialVelocity() {
    return getVelocity() * METERS_PER_SEC_PER_MOTOR_ROT;
  }

  public void setTargetVelocity(double targetVelRPS) {
    this.targetVelRPS = targetVelRPS;
  }

  public void stop() {
    setTargetVelocity(0.0);
  }

  @Override
  public void periodic() {
  }
}
