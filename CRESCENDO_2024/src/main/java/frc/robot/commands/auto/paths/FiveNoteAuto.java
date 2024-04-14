
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
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.actions.AutoAngleSnap;
import frc.robot.commands.auto.actions.PersueAndIntakeNote;
import frc.robot.commands.auto.actions.PersueAndIntakeNoteForShooter;
import frc.robot.commands.handoffs.HandoffFromIntakeToShooter;
import frc.robot.commands.shooter.SpoolShooterForSpeakerShot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class FiveNoteAuto extends SequentialCommandGroup {
  /** Creates a new FiveNoteAuto. */
  public FiveNoteAuto(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    var centerToBottomNote = PathPlannerPath.fromPathFile("Center_to_bottom_note");
    var bottomNoteBackup = PathPlannerPath.fromPathFile("CenterNotebackup");
    var a3_b5 = PathPlannerPath.fromPathFile("A3-B5");
    var b5_speaker = PathPlannerPath.fromPathFile("B5-AgainstSpeaker");


    addCommands(
      new StationaryShootFromAnywhere(shooter, drivetrain),
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false),
      AutoBuilder.pathfindThenFollowPath(centerToBottomNote, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNote(vision, shooter, intake, drivetrain),
      AutoBuilder.pathfindThenFollowPath(bottomNoteBackup, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS)
        .alongWith(
          new HandoffFromIntakeToShooter(shooter, intake, false)
          .andThen(
            new SpoolShooterForSpeakerShot(shooter, false)
          )
        ),
      new StationaryShootFromAnywhere(shooter, drivetrain),
     new ConditionalCommand(
      new AutoAngleSnap(Rotation2d.fromDegrees(40), drivetrain), 
      new AutoAngleSnap(Rotation2d.fromDegrees(180 - 40), drivetrain), 
      () -> {
          Optional<Alliance> allyOpt = DriverStation.getAlliance();
          return allyOpt.isPresent() && allyOpt.get() == Alliance.Blue;
      }),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      new ConditionalCommand(
      new AutoAngleSnap(Rotation2d.fromDegrees(70), drivetrain), 
      new AutoAngleSnap(Rotation2d.fromDegrees(180 - 70), drivetrain), 
      () -> {
          Optional<Alliance> allyOpt = DriverStation.getAlliance();
          return allyOpt.isPresent() && allyOpt.get() == Alliance.Blue;
      }),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      AutoBuilder.pathfindThenFollowPath(a3_b5, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new SpoolShooterForSpeakerShot(shooter, false),
      AutoBuilder.pathfindThenFollowPath(b5_speaker, AutoConstants.PATHFIND_TO_AUTOPATH_START_CONSTRAINTS),
      new StationaryShootFromAnywhere(shooter, drivetrain)
    );
  }
}