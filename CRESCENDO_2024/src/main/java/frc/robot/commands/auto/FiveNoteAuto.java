// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.StationaryShootFromAnywhere;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveNoteAuto extends SequentialCommandGroup {
  /** Creates a new FiveNoteAuto. */
  public FiveNoteAuto(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    var centerToBottomNote = PathPlannerPath.fromPathFile("Center_To_bottom_note");
    var bottomNoteBackup = PathPlannerPath.fromPathFile("CenterNotebackup");
    var a3_b5 = PathPlannerPath.fromPathFile("A3-B5");
    var b5_speaker = PathPlannerPath.fromPathFile("B5-AgainstSpeaker");


    addCommands(
      new StationaryShootFromAnywhere(shooter, drivetrain),

      AutoBuilder.followPath(centerToBottomNote),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(bottomNoteBackup),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      new AutoAngleSnap(Rotation2d.fromDegrees(50), drivetrain), // roman fix this if its wrong
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      new AutoAngleSnap(Rotation2d.fromDegrees(90), drivetrain),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),

      AutoBuilder.followPath(a3_b5),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(b5_speaker),
      new StationaryShootFromAnywhere(shooter, drivetrain)
    );
  }
}
