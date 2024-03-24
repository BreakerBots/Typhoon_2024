
package frc.robot.commands.auto.paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision;
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
      AutoBuilder.followPath(centerToBottomNote),
      new PersueAndIntakeNote(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(bottomNoteBackup)
        .alongWith(
          new HandoffFromIntakeToShooter(shooter, intake, false)
          .andThen(
            new SpoolShooterForSpeakerShot(shooter, false)
          )
        ),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      new AutoAngleSnap(Rotation2d.fromDegrees(50), drivetrain), 
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      new AutoAngleSnap(Rotation2d.fromDegrees(90), drivetrain),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      AutoBuilder.followPath(a3_b5),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new SpoolShooterForSpeakerShot(shooter, false),
      AutoBuilder.followPath(b5_speaker),
      new StationaryShootFromAnywhere(shooter, drivetrain)
    );
  }
}