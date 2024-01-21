// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

public class ShooterCarrage extends SubsystemBase {
  /** Creates a new PizzaBox. */
  private TalonFX pitchMotor;
  private WPI_TalonSRX hopperMotor;
  private Rotation2d targetPitch;
  private CarrageHopperState hopperState;
  private static final BreakerBeamBreak beamBreak = new BreakerBeamBreak(0, true);
  public ShooterCarrage() {

  }


  

  public static boolean hasNote() {
    return beamBreak.isBroken();
  }

  public boolean isAtTargetState() {
    return true;
  }

  public void setCarragePitchTarget(Rotation2d target) {

  }

  public void setHopperState(CarrageHopperState state) {

  }

  public Command setCarragePitchTargetCommand(Rotation2d target, boolean waitForSuccess) {
    return new FunctionalCommand(() -> {setCarragePitchTarget(target);}, () -> {}, (Boolean interupted) -> {}, () -> {return !waitForSuccess || isAtTargetState();}, this);
  }

  public InstantCommand setHopperStateCommand(CarrageHopperState state) {
    return new InstantCommand(() -> {setHopperState(state);});
  }

  public ParallelCommandGroup setCarrageStateCommand(Rotation2d pitchTarget, CarrageHopperState newHopperState, boolean waitForSuccess) {
    return setCarragePitchTargetCommand(targetPitch, waitForSuccess).alongWith(setHopperStateCommand(newHopperState));
  }
  

  public static class CarragePitchPresets {
    public static final Rotation2d STOW = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d PASTA_ROLLER_HANDOFF = Rotation2d.fromDegrees(90.0);
    public static final Rotation2d INTAKE_FROM_HUMAN_PLAYER = Rotation2d.fromDegrees(55.0);
    public static final Rotation2d EJECT_NOTE = Rotation2d.fromDegrees(45.0);
  }

  public static enum CarrageHopperState {
    INTAKE,
    EXTAKE,
    NEUTRAL;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
