// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PastaRoller extends SubsystemBase {
  /** Creates a new PastaRoller. */
  private WPI_TalonSRX innerRollerMotor, outerRollerMotor;
  private PastaRollerState curState;
  public PastaRoller() {

  }

  public void setState(PastaRollerState state) {
    curState = state;
  }

  public InstantCommand setStateCommand(PastaRollerState state) {
    return new InstantCommand(() -> {setState(state);}, this);
  }

  public static enum PastaRollerState {
    INTAKE(0.2, 0.5),
    EXTAKE(0.2, -0.8),
    NEUTRAL(0.0, 0.0);
    private double innerRollerDutyCycle;
    private double outerRollerDutyCycle;
    private PastaRollerState(double innerRollerDutyCycle, double outerRollerDutyCycle) {
      this.innerRollerDutyCycle = innerRollerDutyCycle;
      this.outerRollerDutyCycle = outerRollerDutyCycle;
    }

    public double getInnerRollerDutyCycle() {
        return innerRollerDutyCycle;
    }

    public double getOuterRollerDutyCycle() {
        return outerRollerDutyCycle;
    }
  }
  @Override
  public void periodic() {
    innerRollerMotor.set(curState.getInnerRollerDutyCycle());
    outerRollerMotor.set(curState.getOuterRollerDutyCycle());
  }
}