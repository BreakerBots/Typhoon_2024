// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilCommandWithConditionSatisfactionDuration extends Command {
  /** Creates a new WaitUntilCommandWithConditionSatisfactionDuration. */
  private BooleanSupplier condition;
  private double requiredSatisfactionDuration;
  private Timer timer;
  public WaitUntilCommandWithConditionSatisfactionDuration(BooleanSupplier condition, double requiredSatisfactionDuration) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (condition.getAsBoolean()) {
      timer.restart();
    } else {
      timer.stop();
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return condition.getAsBoolean() && timer.hasElapsed(requiredSatisfactionDuration);
  }
}
