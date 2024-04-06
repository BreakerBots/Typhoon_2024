// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.handoffs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandoffFromIntakeToShooter extends SequentialCommandGroup {
  /** Creates a new HandoffFromShooterToIntake. */
  public HandoffFromIntakeToShooter(Shooter shooter, Intake intake, boolean retractAtEnd) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new ParallelCommandGroup(
        //     new InstantCommand(() -> shooter.setState(ShooterState.STOW), shooter)
        //         .andThen(new WaitUntilCommand(shooter::isAtAngleGoal)),
        //     intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true)
        // ),
        intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, true),
        new InstantCommand(() -> shooter.setState(ShooterState.INTAKE_TO_SHOOTER_HANDOFF), shooter),
        new WaitUntilCommand(shooter::hasNote),
        intake.setStateCommand(retractAtEnd ? IntakeState.RETRACTED_NEUTRAL : IntakeState.EXTENDED_NEUTRAL, false),
        new InstantCommand(() -> shooter.setState(ShooterState.SMART_SPOOL), shooter)
    );
  }
}
