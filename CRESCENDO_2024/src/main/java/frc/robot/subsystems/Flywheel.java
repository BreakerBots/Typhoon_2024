// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private TalonFX flywheelA, flywheelB;
  public Flywheel() {
    
  } 

  public boolean isAtTargetVelocity(FlywheelPresisionType presisionType) {

  }

  public double getVelocityError() {
    
  }

  public double getVelocity() {
    
  }

  public double getTangetialVelocity() {
    
  }

  public void setMode(FlywheelMode mode) {

  }

  public static enum FlywheelPresisionType {
    COARSE,
    FINE,
    NONE
  }

  public static enum FlywheelMode {
    NEUTRAL,
    INTAKE,
    PASTA_ROLLER_HANDOFF,
    EJECT_NOTE,
    SHOOT_SPEAKER
  }

  @Override
  public void periodic() {
    
  }
}
