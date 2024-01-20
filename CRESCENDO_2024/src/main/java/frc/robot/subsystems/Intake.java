// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX rollerMotor;  
  private TalonFX pivotMotor;
  private IntakeState targetState;
 

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new WPI_TalonSRX(ROLLER_MOTOR_ID);
    pivotMotor = new TalonFX(PIVOT_MOTOR_ID);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = ;//TODO
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    pivotConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = true;

    pivotConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    pivotConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = true;


  }

  public boolean isAtTargetState() {
    return true;
  }

  public void setState(IntakeState stateToSet) {
    targetState = stateToSet;
  }
  
  public Command setStateCommand(IntakeState stateToSet, boolean waitForSuccess) {
    return new FunctionalCommand(() -> {setState(stateToSet);}, () -> {}, (Boolean interupted) -> {}, () -> {return !waitForSuccess || isAtTargetState();}, this);
  }

  public static enum IntakeState {
    EXTENDED_INTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.INTAKEING),
    EXTENDED_EXTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.EXTAKEING),
    EXTENDED_NEUTRAL(IntakePivotState.EXTENDED, IntakeRollerState.NEUTRAL),
    RETRACTED_NEUTRAL(IntakePivotState.RETRACTED, IntakeRollerState.NEUTRAL);
    
    private IntakePivotState pivotState;
    private IntakeRollerState rollerState;
    private IntakeState(IntakePivotState piviotState, IntakeRollerState rollerState) {
      this.pivotState = piviotState;
      this.rollerState = rollerState;
    }
  }

  public static enum IntakePivotState {
    EXTENDED(0.6),
    RETRACTED(-0.6);

    private IntakePivotState(double motorDutyCycle) {
      
    }
  }

  public static enum IntakeRollerState {
    INTAKEING,
    EXTAKEING,
    NEUTRAL
  }



  @Override
  public void periodic() {
   
  }
}
