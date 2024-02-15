// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeConstantTest extends Command {
  /** Creates a new IntakeConstantTest. */
  private Intake intake;
  private Shooter shooter;
  public IntakeConstantTest(Intake intake, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setState(IntakeState.EXTENDED_INTAKEING);
    shooter.setHopSpeed(-1.0);
    shooter.setFlywheelSpeed(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeState.EXTENDED_NEUTRAL);
    shooter.setHopSpeed(0.0);
    shooter.setFlywheelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
