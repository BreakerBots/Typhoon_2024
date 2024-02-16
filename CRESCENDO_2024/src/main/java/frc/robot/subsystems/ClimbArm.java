// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

public class ClimbArm extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonFX motor;
  private boolean homed;
  private boolean positionControl;
  private DutyCycleOut dutyCycleRequest;
  private MotionMagicVoltage positionRequest;
  public ClimbArm(int motorID, String  motorBus) {
    motor = new TalonFX(motorID, motorBus);
    positionControl = false;
    homed = false;
    dutyCycleRequest = new DutyCycleOut(0.0);
    
  }

  public void setTargetPositionRotations(double targetPos) {
    if (homed) {
      positionControl = true;
      positionRequest.withPosition(targetPos);
    } else {
      String warnStr = "WARNING: CLIMB POS CTRL CANT BE USED BEFORE HOMEING!";
      DriverStation.reportWarning(warnStr, false);
    }
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
    if (RobotState.isDisabled()) {
      motor.set(0.0);
    }
    if (positionControl) {
      motor.setControl(positionRequest);
    } else {
      motor.setControl(dutyCycleRequest);
    }
  }
}
