// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.AmpBar.AmpBarState;
import frc.robot.subsystems.Intake.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmp extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  public ScoreInAmp(Intake intake, AmpBar ampBar) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.AMP_NEUTRAL, true)
        .alongWith(ampBar.setStateCommand(AmpBarState.EXTENDED, true)),
      intake.setStateCommand(IntakeState.AMP_EXTAKING, false),
       new WaitUntilCommndWithFallingEdgeDelayAndTimeout(() ->{return !intake.hasNote();}, 1.0, 3.0),
      intake.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false),
      ampBar.setStateCommand(AmpBarState.RETRACTED, false)
    );
  }
}
