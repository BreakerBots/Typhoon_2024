// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.actions.PersueAndIntakeNoteForShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetterFiveNote extends SequentialCommandGroup {
  /** Creates a new BetterFiveNote. */
  public BetterFiveNote(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var a2ToA3 = PathPlannerPath.fromPathFile("A2 to A3");
    var a3ToB4 = PathPlannerPath.fromPathFile("A3 to B4");
    var b4ToShoot = PathPlannerPath.fromPathFile("B4 to Shoot");
    var shootToB5 = PathPlannerPath.fromPathFile("Shoot to B5"); 
    var b5ToShoot = PathPlannerPath.fromPathFile("B5 to Shoot");
    addCommands(
      new StationaryShootFromAnywhere(shooter, drivetrain).alongWith(intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false)),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.followPath(a2ToA3),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.followPath(a3ToB4),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(b4ToShoot),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.followPath(shootToB5),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(b5ToShoot)

    );
  }
}
