// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootManualAllign extends SequentialCommandGroup {
  /** Creates a new ShootManualAllign. */
  public ShootManualAllign(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> shooter.setActiveTarget(Constants.ShooterConstants.MANUAL_SPEAKER_SHOT_FIREING_SOLUTION_SUPPLIER), shooter),
      new InstantCommand(() -> shooter.setState(ShooterState.TRACK_TARGET)),
      new WaitUntilCommand(shooter::isAtGoal),
      new InstantCommand(() -> shooter.setState(ShooterState.SHOOT_TO_TARGET)),
      new WaitUntilCommndWithFallingEdgeDelayAndTimeout(() -> {return !shooter.hasNote();}, 0.5, 3.0),
      new InstantCommand(() -> shooter.setState(ShooterState.TRACK_TARGET_IDLE))
    );
    
  }
}
