// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllignToAmp extends SequentialCommandGroup {
  /** Creates a new AllignToAmp. */
  public AllignToAmp(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      AutoBuilder.pathfindToPose(new Pose2d(1.82, 7.5, Rotation2d.fromDegrees(90.0)), new PathConstraints(1.5, 8.0, 5.0, 5.0)),
      new MoveToPose(drive, new Pose2d(1.82, 7.71, Rotation2d.fromDegrees(90.0)), new TrapezoidProfile.Constraints(0.75, 2.0), new TrapezoidProfile.Constraints(2.0, 5.0), true)
    );
  }
}
