// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Vision;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.actions.AutoAngleSnap;
import frc.robot.commands.auto.actions.PersueAndIntakeNoteForShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveNoteAuto extends SequentialCommandGroup {
  /** Creates a new FiveNoteAuto. */
  public FiveNoteAuto(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    var A2ToA1 = PathPlannerPath.fromPathFile("A2ToA1");
    var A1ToA3 = PathPlannerPath.fromPathFile("A1ToA3");
    var A3ToB4 = PathPlannerPath.fromPathFile("A3ToB4");
    var B4ToSpeakerHigh = PathPlannerPath.fromPathFile("B4ToSpeakerHigh");
    var SpeakerHighToB3 = PathPlannerPath.fromPathFile("SpeakerHighToB3UnderChain");
    var B3ToSpeakerHigh = PathPlannerPath.fromPathFile("B3ToSpeakerHighUnderChain");
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
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.pathfindThenFollowPath(A2ToA1, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.pathfindThenFollowPath(A1ToA3, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.pathfindThenFollowPath(A3ToB4, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.pathfindThenFollowPath(B4ToSpeakerHigh, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      // CUT THESE LINES OUT IF AUTOPATH TAKES TOO LONG
      AutoBuilder.pathfindThenFollowPath(SpeakerHighToB3, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.pathfindThenFollowPath(B3ToSpeakerHigh, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new StationaryShootFromAnywhere(shooter, drivetrain)
    );
  }
}
