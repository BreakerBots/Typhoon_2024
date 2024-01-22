// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.PastaRoller.PastaRollerState;
import frc.robot.subsystems.ShooterCarrage.CarrageHopperState;
import frc.robot.subsystems.ShooterCarrage.CarragePitchMode;

/** Add your docs here. */
public enum SuperstructureState {
    /*Engagued by a disable event to ensure te robot does not snap back to its pre-disabled state if any changes occur */
    ROBOT_NEUTRAL(IntakeState.NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.HOLD_ARBITRARY, CarrageHopperState.NEUTRAL, FlywheelState.NEUTRAL),
    STOW(IntakeState.RETRACTED_NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.STOW, CarrageHopperState.NEUTRAL, FlywheelState.NEUTRAL),

    INTAKE_FROM_GROUND_PREP(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.STOW, CarrageHopperState.INTAKE, FlywheelState.INTAKE),
    INTAKE_FROM_GROUND(IntakeState.EXTENDED_INTAKEING, PastaRollerState.NEUTRAL, CarragePitchMode.STOW, CarrageHopperState.INTAKE, FlywheelState.INTAKE),

    INTAKE_FROM_HUMAN_PLAYER(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.INTAKE_FROM_HUMAN_PLAYER, CarrageHopperState.INTAKE, FlywheelState.INTAKE),

    HANDOFF_TO_PASTA_ROLLER_PREP(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.INTAKE, CarragePitchMode.PASTA_ROLLER_HANDOFF, CarrageHopperState.NEUTRAL, FlywheelState.PASTA_ROLLER_HANDOFF),
    HANDOFF_TO_PASTA_ROLLER(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.INTAKE, CarragePitchMode.PASTA_ROLLER_HANDOFF, CarrageHopperState.EXTAKE, FlywheelState.PASTA_ROLLER_HANDOFF),
    HANDBACK_FROM_PASTA_ROLLER(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.EXTAKE_REVERSE, CarragePitchMode.PASTA_ROLLER_HANDOFF, CarrageHopperState.INTAKE, FlywheelState.INTAKE),
    EXTAKE_FROM_PASTA_ROLLER(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.EXTAKE, CarragePitchMode.STOW, CarrageHopperState.INTAKE, FlywheelState.INTAKE),

    SHOOT_INTO_SPEAKER_PREP(IntakeState.EXTENDED_NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.SHOOT_SPEAKER, CarrageHopperState.NEUTRAL, FlywheelState.SHOOT_SPEAKER),
    SHOOT_INTO_SPEAKER(),

    EJECT_NOTE_PREP(),
    EJECT_NOTE(),

    CLIMB_EXTEND(),
    CLIMB_RETRACT();
    final IntakeState intakeState; 
    final PastaRollerState pastaRollerState; 
    final CarragePitchMode shooterCarragePitchMode; 
    final CarrageHopperState shooterCarrageHopperState; 
    final FlywheelState flywheelState;
    private SuperstructureState(IntakeState intakeState, PastaRollerState pastaRollerState, CarragePitchMode shooterCarragePitchMode, CarrageHopperState shooterCarrageHopperState, FlywheelState flywheelState) {

    }

    public FlywheelState getFlywheelState() {
        return flywheelState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public PastaRollerState getPastaRollerState() {
        return pastaRollerState;
    }

    public CarrageHopperState getShooterCarrageHopperState() {
        return shooterCarrageHopperState;
    }

    public CarragePitchMode getShooterCarragePitchMode() {
        return shooterCarragePitchMode;
    }


}
