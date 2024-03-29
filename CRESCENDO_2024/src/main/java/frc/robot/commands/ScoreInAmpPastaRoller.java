// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PastaRoller;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.PastaRoller.PastaRollerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpPastaRoller extends SequentialCommandGroup {
  /** Creates a new HandoffToPastaRollerTest. */
  public ScoreInAmpPastaRoller(Intake intake, PastaRoller pastaRoller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.RETRACTED_NEUTRAL, true),
      intake.setStateCommand(IntakeState.RETRACTED_EXTAKEING, false),
      pastaRoller.setStateCommand(PastaRollerState.EXTAKE),
      new WaitUntilCommndWithFallingEdgeDelayAndTimeout(() ->{return !intake.hasNote();}, 1.5, 3.0),
      intake.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false),
      pastaRoller.setStateCommand(PastaRollerState.NEUTRAL)
      );
  }
}
