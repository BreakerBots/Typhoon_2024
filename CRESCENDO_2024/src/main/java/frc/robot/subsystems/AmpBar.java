// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpBar extends SubsystemBase { // 👍
  enum AmpBarState {
    RETRACTED(-0.7),
    EXTENDED(0.7);

    private double motorDutyCycle;

    private AmpBarState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
      return motorDutyCycle;
    }
  }

  private CANSparkFlex sparkFlex;
  private CANcoder canCoder;
  private AmpBarState currentState;

  private static final double MAX_ANGLE_THRESHOLD = 60.0; // temporary
  private static final double MIN_ANGLE_THRESHOLD = 10.0; // temporary

  public Command setStateCommand(AmpBarState desiredState, boolean waitToFinish) {
    return new FunctionalCommand(
      () -> {
        setState(desiredState);
      }, 
      () -> {}, 
      (i) -> {},
      () -> !waitToFinish || isAtTargetAngle(),
      this
    );
  }

  public void setState(AmpBarState state) {
    currentState = state;
  }

  public boolean isAtTargetAngle() {
    double value = canCoder.getAbsolutePosition().getValueAsDouble();
    return switch (currentState) {
      case EXTENDED -> value > MAX_ANGLE_THRESHOLD;
      case RETRACTED -> value < MIN_ANGLE_THRESHOLD;
    };
  }

  /** Creates a new AmpBar. */
  public AmpBar() {
    sparkFlex = new CANSparkFlex(0, MotorType.kBrushless); // TOOD fill in device id
    canCoder = new CANcoder(0); // TODO fill device id
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isAtTargetAngle()) {
      sparkFlex.set(currentState.motorDutyCycle);
    }
  }
}
