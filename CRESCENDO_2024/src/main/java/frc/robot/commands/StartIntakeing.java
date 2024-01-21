// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.commands.shooter.SetShooterState.ShooterPresets;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterCarrage;
import frc.robot.subsystems.Intake.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartIntakeing extends SequentialCommandGroup {
  /** Creates a new StartIntakeing. */
  public StartIntakeing(Intake intake, ShooterCarrage carrage, Flywheel flywheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterState(flywheel, carrage, ShooterPresets.INTAKE_PREP, true).alongWith(intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true)),
      new SetShooterState(flywheel, carrage, ShooterPresets.INTAKE, false).alongWith(intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, false))
    );
  }
}
