// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerGenericGamepad;

public class BreakerGamepadTimedRumbleCommand extends Command {
  /** Creates a new BreakerGamepadTimedRumbleCommand. */

  private BreakerGenericGamepad gamepad;
  private double leftRumble, rightRumble, timeoutSeconds;
  private final Timer timer = new Timer();
  public BreakerGamepadTimedRumbleCommand(BreakerGenericGamepad gamepad, double timeoutSeconds, double leftRumble, double rightRumble) {
    this.gamepad = gamepad;
    this.leftRumble = leftRumble;
    this.rightRumble = rightRumble;
    this.timeoutSeconds = timeoutSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    gamepad.setMixedRumble(leftRumble, rightRumble);
  }

  @Override
  public void execute() {
    gamepad.setMixedRumble(leftRumble, rightRumble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gamepad.clearRumble();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeoutSeconds);
  }
}
