// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SuperstructureState;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToShooterHandoff extends SequentialCommandGroup {
  /** Creates a new IntakeToShooterHandoff. */
  public IntakeToShooterHandoff(Superstructure superstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_EXTENDED_HOLD, true),
      new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_TO_SHOOTER_HANDOFF, false),
      new WaitUntilCommand(() -> {return !superstructure.intakeHasNote() && superstructure.shooterCarrageHasNote();}),
      new SetSuperstructureState(superstructure, SuperstructureState.SHOOTER_HOLD_NOTE, false)
    );
  }
}
