// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

public class PastaRoller extends SubsystemBase {
  /** Creates a new PastaRoller. */
  private TalonFX rollerMotor;
  private PastaRollerState state;
  public PastaRoller() {
    state = PastaRollerState.NEUTRAL;
    rollerMotor = new TalonFX(60);
    
  }

  public void setState(PastaRollerState state) {
    this.state = state;
    rollerMotor.set(state.getDutyCycle());
  }

  public InstantCommand setStateCommand(PastaRollerState state) {
    return new InstantCommand(() -> {setState(state);}, this);
  }

  public static enum PastaRollerState {
    EXTAKE(0.45),
    NEUTRAL(0.0);
    private double dutyCycle;
    private PastaRollerState(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }

    public double getDutyCycle() {
        return dutyCycle;
    }
  }
  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      state = PastaRollerState.NEUTRAL;
    }
    rollerMotor.set(state.getDutyCycle());
  }
}
