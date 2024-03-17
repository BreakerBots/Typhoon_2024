// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveShootOneSource extends SequentialCommandGroup {
  /** Creates a new LeaveShootOne. */
  public LeaveShootOneSource(Drive drive, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var leaveToShoot = PathPlannerPath.fromPathFile("Source_Leave_To_Shoot");
    addCommands(
      AutoBuilder.followPath(leaveToShoot),
      new StationaryShootFromAnywhere(shooter, drive)
    );
  }
}
