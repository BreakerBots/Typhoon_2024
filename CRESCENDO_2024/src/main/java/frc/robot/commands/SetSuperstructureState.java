// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperstructureState;
import frc.robot.subsystems.Superstructure;

public class SetSuperstructureState extends Command {
  /** Creates a new SetSuperstructureState. */
  private Superstructure superstructure;
  private SuperstructureState state;
  private boolean waitForSuccess;
  public SetSuperstructureState(Superstructure superstructure, SuperstructureState state, boolean waitForSuccess) {
    addRequirements(superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    superstructure.setSuperstructureState(state);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return superstructure.isAtTargetState() || !waitForSuccess;
  }
}
