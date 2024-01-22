// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperstructureState;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromGround extends SequentialCommandGroup {
  /** Creates a new IntakeFromGround. */
  public IntakeFromGround(Superstructure superstructure) {
    addCommands(
      new InstantCommand(() -> {if(superstructure.hasNote()) {this.cancel();}}),
      new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_FROM_GROUND_PREP, true),
      new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_FROM_GROUND, false)
    );
  }
}
