// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbArm extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonFX motor;
  private boolean homed;
  public ClimbArm() {}

  public void setTargetPositionRotations(double targetPos) {
    
  }

  public void setDutyCycle(double dutyCycle) {
    
  }

  public void setHomeingSpeed() {
    setDutyCycle(-0.3);
  }

  public double getMotorCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  public void resetEncoder() {
    motor.setPosition(0.0);
  }

  public double getPosition() {
    return motor.getRotorPosition().getValue();
  }

  public boolean isHomed() {
    return homed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
