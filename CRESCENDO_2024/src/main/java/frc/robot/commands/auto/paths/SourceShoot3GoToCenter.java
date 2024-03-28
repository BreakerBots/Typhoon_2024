// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.actions.PersueAndIntakeNoteForShooter;
import frc.robot.commands.shooter.SpoolShooterForSpeakerShot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceShoot3GoToCenter extends SequentialCommandGroup {
  /** Creates a new SourceShoot3GoToCenter. */
  public SourceShoot3GoToCenter(Shooter shooter, Drive drivetrain, Intake intake, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var goOut = PathPlannerPath.fromPathFile("A1-B1");
    var goBack = PathPlannerPath.fromPathFile("B1-Speaker");
    var goOutToSecondNote = PathPlannerPath.fromPathFile("Speaker_To_B2");
    var gobackFromSecondNote = PathPlannerPath.fromPathFile("B2_To_Speaker");
    

    addCommands(
      new StationaryShootFromAnywhere(shooter, drivetrain).andThen(intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false)),
      AutoBuilder.followPath(goOut),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(goBack),
      new StationaryShootFromAnywhere(shooter, drivetrain),
      AutoBuilder.followPath(goOutToSecondNote),
      new PersueAndIntakeNoteForShooter(vision, shooter, intake, drivetrain),
      AutoBuilder.followPath(gobackFromSecondNote),
      new StationaryShootFromAnywhere(shooter, drivetrain)
    );
  }
}
