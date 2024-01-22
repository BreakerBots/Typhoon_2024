// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Flywheel.FlywheelMode;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.PastaRoller.PastaRollerState;
import frc.robot.subsystems.ShooterCarrage.CarrageHopperState;
import frc.robot.subsystems.ShooterCarrage.CarragePitchMode;

/** Add your docs here. */
public enum SuperstructureState {
    /*Engagued by a disable event to ensure te robot does not snap back to its pre-disabled state if any changes occur */
    ROBOT_NEUTRAL(IntakeState.NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.HOLD_ARBITRARY, CarrageHopperState.NEUTRAL, FlywheelMode.NEUTRAL),

    STOW(IntakeState.RETRACTED_NEUTRAL, PastaRollerState.NEUTRAL, CarragePitchMode.STOW, CarrageHopperState.NEUTRAL, FlywheelMode.NEUTRAL),
    INTAKE_FROM_GROUND(IntakeState.EXTENDED_INTAKEING, PastaRollerState.NEUTRAL, CarragePitchMode.STOW, CarrageHopperState.INTAKE, FlywheelMode.INTAKE),
    HANDOFF_TO_PASTA_ROLLER(),
    INTAKE_FROM_HUMAN_PLAYER(),
    EXTAKE_FROM_PASTA_ROLLER(),
    SHOOT_INTO_SPEAKER(),
    EJECT_NOTE(),
    CLIMB_EXTEND(),
    CLIMB_RETRACT();
    final IntakeState intakeState; 
    final PastaRollerState pastaRollerState; 
    final CarragePitchMode shooterCarragePitchMode; 
    final CarrageHopperState shooterCarrageHopperState; 
    final FlywheelMode flywheelMode;
    private SuperstructureState(IntakeState intakeState, PastaRollerState pastaRollerState, CarragePitchMode shooterCarragePitchMode, CarrageHopperState shooterCarrageHopperState, FlywheelMode flywheelMode) {

    }

    public FlywheelMode getFlywheelMode() {
        return flywheelMode;
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
