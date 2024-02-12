// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbArm extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonFX motor;
  private boolean homed;
  private boolean positionControl;
  private DutyCycleOut dutyCycleRequest;
  private MotionMagicVoltage positionRequest;
  public ClimbArm(int motorID, String  motorBus) {

  }

  public void setTargetPositionRotations(double targetPos) {
    positionControl = true;
    positionRequest.withPosition(targetPos);
    
  }

  public void setDutyCycle(double dutyCycle) {
    positionControl = false;
    dutyCycleRequest.withOutput(dutyCycle);
  }

  public void setHomeingSpeed() {
    setDutyCycle(-0.3);
  }

  public void setSoftLevelSpeed() {
    setDutyCycle(-0.1);
  }

  public double getMotorCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  public void home() {
    motor.setPosition(0.0);
    homed = true;
  }
 
  public double getPosition() {
    return motor.getRotorPosition().getValue();
  }

  public boolean isAtTargetPosition() {
    return MathUtil.isNear(positionRequest.Position, getPosition(), getMotorCurrent());
  }

  public boolean isHomed() {
    return homed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
