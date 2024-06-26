// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.other;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.actions.AutoAngleSnap;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// This autopath delays scoring into the speaker, so that it can yield way for other more active teams who are scoring in the autonomous period

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomSpeakerDelayShootThenExit extends SequentialCommandGroup {
  /** Creates a new BottomSpeakerDelayShootThenExit. */
  public BottomSpeakerDelayShootThenExit(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    var A1ToB1 = PathPlannerPath.fromPathFile("A1ToB1");

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(5),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      new ConditionalCommand(
          new AutoAngleSnap(Rotation2d.fromDegrees(0.0), drivetrain),
          new AutoAngleSnap(Rotation2d.fromDegrees(180.0), drivetrain), () -> {
          Optional<Alliance> allyOpt = DriverStation.getAlliance();
          return allyOpt.isPresent() && allyOpt.get() == Alliance.Blue;
      }),
      AutoBuilder.pathfindThenFollowPath(A1ToB1, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS) //ends near the middle for a head start
    );
  }
}
