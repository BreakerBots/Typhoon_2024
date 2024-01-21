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

  public void setVelocity(double velocityRotationsPerSecond) {

  }

  public Command setVelocityCommand(double velocityRotationsPerSecond, FlywheelPresisionType presisionType) {
    return new FunctionalCommand(() -> {setVelocity(velocityRotationsPerSecond);}, () -> {}, (Boolean interupted) -> {}, () -> {return isAtTargetVelocity(presisionType);}, this);
  }

  private InstantCommand setFlywheelToIdleSpeed() {
    return new InstantCommand(() -> {setVelocity(ShooterTarget.SPEAKER.getRequiredFlywheelVelocity() * 0.6);})
  }

  public Command stopFlywheel(FlywheelPresisionType presisionType) {
    return setVelocityCommand(0.0, presisionType);
  }

  public ConditionalCommand conditionalIdleFlywheel() {
    return new ConditionalCommand(setFlywheelToIdleSpeed(), stopFlywheel(FlywheelPresisionType.NONE), ShooterCarrage::hasNote);
  }

  public static enum FlywheelPresisionType {
    COARSE,
    FINE,
    NONE


  }

  public static class FlywheelPresets {
    public static final double NEUTRAL = 0.0;
    public static final double HANDOFF_FROM_INTAKE = dutyCycleToRPS(0.3);
    public static final double HANDOFF_TO_PASTA_ROLLER = dutyCycleToRPS(0.6);
    public static final double EJECT_NOTE = dutyCycleToRPS(0.6);
    public static final double MAX_SPEED = 6000;

    private static double dutyCycleToRPS(double dutyCycle) {
      return (dutyCycle * 6380.0)/60.0;
    }
  }

  @Override
  public void periodic() {
    
  }
}
