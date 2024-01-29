// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SuperstructureState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PastaRoller;
import frc.robot.subsystems.ShooterCarrage;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmp extends SequentialCommandGroup {
  /** Creates a new ScoreInAmp. */
  public ScoreInAmp(Superstructure superstructure, Drive drivetrain, PastaRoller pastaRoller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToPose(drivetrain, null, true)
      .alongWith(
        new ShooterToPastaRollerHandoff(superstructure, pastaRoller)
        .onlyIf(pastaRoller::doesNotHaveNote)
      ),
      new SetSuperstructureState(superstructure, SuperstructureState.EXTAKE_FROM_PASTA_ROLLER, false),
      new WaitUntilCommndWithFallingEdgeDelayAndTimeout(pastaRoller::doesNotHaveNote, 2.5, 5),
      new SetSuperstructureState(superstructure, SuperstructureState.STOW, false)
    );
  }
}
