// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbArm;

public class PullDownClimbArmsWhileLevel extends Command {
  /** Creates a new PullDownClimbArms. */
  private ClimbArm left, right;
  public PullDownClimbArmsWhileLevel(ClimbArm left, ClimbArm right) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double leftPos = left.getPosition();
    double rightPos = right.getPosition();
    double lowestPos = Math.min(leftPos, rightPos);
    double lowestDelta =  lowestPos - Constants.ClimbConstants.RETRACTED_POSITION_ROTATIONS;
    left.setTargetPositionRotations(leftPos - lowestDelta);
    right.setTargetPositionRotations(rightPos - lowestDelta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return left.isAtTargetPosition() && right.isAtTargetPosition();
  }
}
