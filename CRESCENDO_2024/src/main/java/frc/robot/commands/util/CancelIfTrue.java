// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class CancelIfTrue extends Command {
  /** Creates a new CancelIfTrue. */
  private BooleanSupplier condition;
  private Command commandToCancel;
  public CancelIfTrue(BooleanSupplier condition, Command commandToCancel) {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (condition.getAsBoolean()) {
      commandToCancel.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
