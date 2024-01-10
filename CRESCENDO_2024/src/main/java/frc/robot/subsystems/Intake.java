// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  BreakerBeamBreak beamBreak;
  public Intake() {

  }

  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public boolean isAtTargetState() {
    return true;
  }

  public void setState(IntakeState stateToSet) {

  }

  public Command setStateCommand(IntakeState stateToSet, boolean waitForSuccess) {
    return new InstantCommand(() -> setState(stateToSet), this)
    .andThen(
      new WaitUntilCommand(this::isAtTargetState)
      .raceWith(
        new WaitCommand(0.0)
      )
    );
  }

  public enum IntakeState {
    INTAKEING_EXTENDED,
    NEUTRAL_EXTENDED,
    EXTAKEING_EXTENDED,
    NEUTRAL_RETRACTED,
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
