// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.shooter.SpoolShooterForSpeakerShot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteAgainstSpeaker extends SequentialCommandGroup {
  /** KNOWN GOOD */
  public ThreeNoteAgainstSpeaker(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) { // 🗿
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var goToB5 = PathPlannerPath.fromPathFile("A3-B5");
    var goAgainstSpeaker = PathPlannerPath.fromPathFile("B5-AgainstSpeaker");
    var initalOffset = PathPlannerPath.fromPathFile("Inital Offset Path");
    
    addCommands(
      new SpoolShooterForSpeakerShot(shooter,false),
      AutoBuilder.followPath(initalOffset),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      // new ConditionalCommand(
      //   new AutoAngleSnap(Rotation2d.fromDegrees(0.0), drivetrain),
      //   new AutoAngleSnap(Rotation2d.fromDegrees(180.0), drivetrain), () -> {
      //     Optional<Alliance> allyOpt = DriverStation.getAlliance();
      //     return allyOpt.isPresent() && allyOpt.get() == Alliance.Blue;
      //   }),

      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.followPath(goToB5),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new SpoolShooterForSpeakerShot(shooter, false),
      AutoBuilder.followPath(goAgainstSpeaker),
      new StationaryShootFromAnywhere(shooter, drivetrain)

    );
  }
}
