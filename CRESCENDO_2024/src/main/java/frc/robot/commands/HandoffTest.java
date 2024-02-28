// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandoffTest extends SequentialCommandGroup {
  /** Creates a new ShooterTest. */
  public HandoffTest(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true),
      intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, false),
      new InstantCommand(() -> {shooter.setState(ShooterState.INTAKE_TO_SHOOTER_HANDOFF);}),
      new WaitUntilCommand(shooter::hasNote),
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false),
      new InstantCommand(() -> {shooter.setState(ShooterState.TRACK_TARGET);})
    );
  }
}
