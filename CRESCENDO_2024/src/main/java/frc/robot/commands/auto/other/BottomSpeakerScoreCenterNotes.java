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
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.actions.AutoAngleSnap;
import frc.robot.commands.auto.actions.PersueAndIntakeNoteForShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomSpeakerScoreCenterNotes extends SequentialCommandGroup {
  /** Creates a new BottomSpeakerScoreMiddleNotes. */
  public BottomSpeakerScoreCenterNotes(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    var bottomSpeakerToB1 = PathPlannerPath.fromPathFile("LowSpeakerShootToB1");
    var B1ToLowSpeaker = PathPlannerPath.fromPathFile("B1SpeakerLow");
    var bottomSpeakerToB2 = PathPlannerPath.fromPathFile("LowSpeakerToB2");
    var B2ToLowSpeaker = PathPlannerPath.fromPathFile("B2-LowSpeaker");
    var bottomSpeakerToB3 = PathPlannerPath.fromPathFile("LowSpeakerToB3");

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StationaryShootFromAnywhere(shooter, drivetrain),
      new ConditionalCommand(
        new AutoAngleSnap(Rotation2d.fromDegrees(0.0), drivetrain),
        new AutoAngleSnap(Rotation2d.fromDegrees(180.0), drivetrain), () -> {
        Optional<Alliance> allyOpt = DriverStation.getAlliance();
          return allyOpt.isPresent() && allyOpt.get() == Alliance.Blue;
      }),
      AutoBuilder.pathfindThenFollowPath(bottomSpeakerToB1, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.pathfindThenFollowPath(B1ToLowSpeaker, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.pathfindThenFollowPath(bottomSpeakerToB2, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.pathfindThenFollowPath(B2ToLowSpeaker, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.pathfindThenFollowPath(bottomSpeakerToB3, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS)
    );
  }
}
