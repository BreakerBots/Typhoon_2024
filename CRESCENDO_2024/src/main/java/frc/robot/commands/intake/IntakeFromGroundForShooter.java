// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromGroundForShooter extends SequentialCommandGroup {
  /** Creates a new IntakeForShooter. */
  public IntakeFromGroundForShooter(Intake intake, Shooter shooter, LED led) {
    addCommands(
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true),
      // new ParallelCommandGroup(
      //   intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true).andThen(
      //     intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, false),
      //     new WaitUntilCommand(intake::hasNote)
      //       .andThen(intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false))
      //       .onlyWhile(()->{return !shooter.isAtAngleGoal();})
      //   ),
      //   new InstantCommand(() -> shooter.setState(ShooterState.STOW))
      //   .andThen(new WaitUntilCommand(shooter::isAtAngleGoal))
      // ),
      new InstantCommand(() -> shooter.setState(ShooterState.INTAKE_TO_SHOOTER_HANDOFF), shooter),
      intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, false),
      new WaitUntilCommand(intake::hasNote),
      led.returnToRestState(),
      new WaitUntilCommand(shooter::hasNote),
      new InstantCommand(() -> shooter.setState(ShooterState.TRACK_TARGET_IDLE)),
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false)
    );
  }
}
