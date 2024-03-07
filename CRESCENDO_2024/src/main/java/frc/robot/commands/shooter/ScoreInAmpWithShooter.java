// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpWithShooter extends SequentialCommandGroup {
  /** Creates a new ShootManualAllign. */
  public ScoreInAmpWithShooter(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> shooter.setActiveTarget(() -> {return new FireingSolution(new Rotation2d(0.0), new BreakerVector2(Rotation2d.fromDegrees(65.0), 30));}), shooter),
      new InstantCommand(() -> shooter.setState(ShooterState.TRACK_TARGET)),
      //new WaitUntilCommand(shooter::isAtGoal),
      new WaitCommand(3.0),
      new InstantCommand(() -> shooter.setState(ShooterState.SHOOT_TO_TARGET)),
      new WaitUntilCommndWithFallingEdgeDelayAndTimeout(() -> {return !shooter.hasNote();}, 0.5, 3.0),
      new InstantCommand(() -> shooter.setState(ShooterState.TRACK_TARGET_IDLE))
    );
    
  }
}
