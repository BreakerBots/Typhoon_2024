// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SuperstructureState;
import frc.robot.commands.SetSuperstructureState;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromGroundForShooter extends SequentialCommandGroup {
  /** Creates a new IntakeForShooter. */
  public IntakeFromGroundForShooter(Superstructure superstructure, boolean waitFo) {
    
    addCommands(
      new InstantCommand(() -> {if(superstructure.hasNote()) {this.cancel();}}),
      new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_EXTENDED_HOLD, true),
      new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_TO_SHOOTER_HANDOFF, false)
      //NOTE: HOLD NOTE STATE IS NOT SET FROM HERE AS THAT IS HANDELED BY Superstructure.java in its periodic loop
    );
  }
}
