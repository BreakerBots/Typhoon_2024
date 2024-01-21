// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShooterCarrage;
import frc.robot.subsystems.Flywheel.FlywheelPresets;
import frc.robot.subsystems.Flywheel.FlywheelPresisionType;
import frc.robot.subsystems.ShooterCarrage.CarrageHopperState;
import frc.robot.subsystems.ShooterCarrage.CarragePitchPresets;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterState extends ParallelCommandGroup {
  /** Creates a new SetShooterState. */
  public SetShooterState(Flywheel flywheel, ShooterCarrage carrage, double flywheelTargetVel, Rotation2d carragePitchTarget, CarrageHopperState carrageHopperState, boolean carrageWaitForSuccess, FlywheelPresisionType flywheelPresisionType) {
    addCommands(
      flywheel.setVelocityCommand(flywheelTargetVel, flywheelPresisionType),
      carrage.setCarrageStateCommand(carragePitchTarget, carrageHopperState, carrageWaitForSuccess)
    );
  }

  public SetShooterState(Flywheel flywheel, ShooterCarrage carrage, ShooterPresets presetToSet, boolean waitForSuccess) {
    this(flywheel, carrage, presetToSet.getFlywheelTargetVel(), presetToSet.getCarragePitchTarget(), presetToSet.getCarrageHopperState(), waitForSuccess, presetToSet.getFlywheelPresisionType(waitForSuccess));
  }

  public static enum ShooterPresets {
    STOW_NEUTRAL(FlywheelPresets.NEUTRAL, CarragePitchPresets.STOW, CarrageHopperState.NEUTRAL, FlywheelPresisionType.NONE),
    INTAKE_PREP(FlywheelPresets.HANDOFF_FROM_INTAKE, CarragePitchPresets.STOW, CarrageHopperState.NEUTRAL, FlywheelPresisionType.COARSE),
    INTAKE(FlywheelPresets.HANDOFF_FROM_INTAKE, CarragePitchPresets.STOW, CarrageHopperState.INTAKE, FlywheelPresisionType.COARSE),
    PASTA_ROLLER_HANDOFF(FlywheelPresets.HANDOFF_TO_PASTA_ROLLER, CarragePitchPresets.PASTA_ROLLER_HANDOFF, CarrageHopperState.EXTAKE, FlywheelPresisionType.COARSE),
    PASTA_ROLLER_PREP(FlywheelPresets.HANDOFF_TO_PASTA_ROLLER, CarragePitchPresets.PASTA_ROLLER_HANDOFF, CarrageHopperState.NEUTRAL, FlywheelPresisionType.COARSE),
    PASTA_ROLLER_NETURAL(FlywheelPresets.NEUTRAL, CarragePitchPresets.PASTA_ROLLER_HANDOFF, CarrageHopperState.NEUTRAL, FlywheelPresisionType.NONE),
    EJECT_NOTE_PREP(FlywheelPresets.EJECT_NOTE, CarragePitchPresets.EJECT_NOTE, CarrageHopperState.NEUTRAL, FlywheelPresisionType.COARSE),
    EJECT_NOTE(FlywheelPresets.EJECT_NOTE, CarragePitchPresets.EJECT_NOTE, CarrageHopperState.EXTAKE, FlywheelPresisionType.COARSE);
    private double flywheelTargetVel;
    private Rotation2d carragePitchTarget;
    private CarrageHopperState carrageHopperState;
    private FlywheelPresisionType requredFlywheelPresision;
    private ShooterPresets(double flywheelTargetVel, Rotation2d carragePitchTarget, CarrageHopperState carrageHopperState, FlywheelPresisionType requredFlywheelPresision) {
      
    }

    public CarrageHopperState getCarrageHopperState() {
        return carrageHopperState;
    }

    public Rotation2d getCarragePitchTarget() {
        return carragePitchTarget;
    }

    public double getFlywheelTargetVel() {
        return flywheelTargetVel;
    }

    public FlywheelPresisionType getFlywheelPresisionType(boolean waitForSuccess) {
      return waitForSuccess ? requredFlywheelPresision : FlywheelPresisionType.NONE;
    }
  }
}
