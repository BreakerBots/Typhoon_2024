// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PersueAndIntakeNote extends SequentialCommandGroup {
  /** Creates a new PersueAndIntakeNoteForShooter. */
  public PersueAndIntakeNote(Vision vision, Shooter shooter, Intake intake, Drive drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, true),
      new InstantCommand(() -> shooter.setState(ShooterState.INTAKE_TO_SHOOTER_HANDOFF), shooter),
      new PersueNote(vision, intake, drivetrain),
      new InstantCommand(() -> shooter.setState(ShooterState.STOW)),
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false)
      

    );
  }
}
