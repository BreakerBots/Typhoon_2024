// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AimToTargetStationary;
import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StationaryShootFromAnywhere extends SequentialCommandGroup {
  /** Creates a new StationaryShootFromAnywhere. */
  public StationaryShootFromAnywhere(Shooter shooter, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> shooter.setActiveTarget(RobotContainer.SPEAKER_TARGET::getFireingSolution)),
      new AimToTargetStationary(shooter, drive),
      new InstantCommand(() -> shooter.setState(ShooterState.SHOOT_TO_TARGET)),
      new WaitUntilCommndWithFallingEdgeDelayAndTimeout(() -> {return !shooter.hasNote();}, 0.5, 3.0),
      new InstantCommand(() -> shooter.setState(ShooterState.TRACK_TARGET_IDLE))
    );
  }
}
