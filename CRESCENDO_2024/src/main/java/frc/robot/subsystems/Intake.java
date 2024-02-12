// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {
  private TalonFX rollerMotor;  
  private TalonFX pivotLeft, pivotRight;
  private IntakeState targetState;
  private CANcoder piviotEncoder;
  private DutyCycleOut piviotDutyCycleControlRequest;
  private Follower pivotFollowerRequest;
  private BreakerBeamBreak beamBreak;
 

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
    pivotLeft = new TalonFX(PIVOT_LEFT_ID);
    pivotRight = new TalonFX(PIVOT_RIGHT_ID);
    
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotLeft.getConfigurator().apply(pivotConfig);
    pivotRight.getConfigurator().apply(pivotConfig);

    BreakerCANCoderFactory.configExistingCANCoder(piviotEncoder, AbsoluteSensorRangeValue.Unsigned_0To1, 0.0,SensorDirectionValue.CounterClockwise_Positive);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
    rollerConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerMotor.getConfigurator().apply(rollerConfig);

    piviotDutyCycleControlRequest = new DutyCycleOut(0.0);
    pivotFollowerRequest = new Follower(PIVOT_LEFT_ID, true);
  }

  public Command setStateCommand(IntakeState stateToSet, boolean waitForSuccess) {
    return new FunctionalCommand(() -> {setState(stateToSet);}, ()-> {}, (Boolean interrupted) -> {}, () -> {return !waitForSuccess||isAtTargetState();}, this);
  }

  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public IntakeState getState() {
    return targetState;
  }

  public boolean isAtTargetState() {
    switch(targetState.getPivotState()) {
      case EXTENDED:
        return isForwardLimitTriggered();
      case RETRACTED:
         return isReverseLimitTriggered();
      case NEUTRAL:
      default:
        return true;
    }
  }

  public void setState(IntakeState stateToSet) {
    targetState = stateToSet;
  }

  public static enum IntakeState {
    EXTENDED_EXTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.EXTAKEING),
    EXTENDED_INTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.INTAKEING),
    EXTENDED_NEUTRAL(IntakePivotState.EXTENDED, IntakeRollerState.NEUTRAL),
    RETRACTED_NEUTRAL(IntakePivotState.RETRACTED, IntakeRollerState.NEUTRAL),
    RETRACTED_EXTAKEING(IntakePivotState.RETRACTED, IntakeRollerState.EXTAKEING),
    RETRACTED_INTAKEING(IntakePivotState.RETRACTED, IntakeRollerState.INTAKEING),
    NEUTRAL(IntakePivotState.NEUTRAL, IntakeRollerState.NEUTRAL);
    
    private IntakePivotState pivotState;
    private IntakeRollerState rollerState;
    private IntakeState(IntakePivotState piviotState, IntakeRollerState rollerState) {
      this.pivotState = piviotState;
      this.rollerState = rollerState;
    }

    public IntakePivotState getPivotState() {
        return pivotState;
    }

    public IntakeRollerState getRollerState() {
        return rollerState;
    }
  }

  public static enum IntakePivotState {
    EXTENDED(0.6),
    RETRACTED(-0.6),
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakePivotState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }

  public static enum IntakeRollerState {
    INTAKEING(-0.5),
    EXTAKEING(0.7),
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakeRollerState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }

  public boolean isForwardLimitTriggered() {
    return piviotEncoder.getAbsolutePosition().getValue() >= PIVIOT_EXTENDED_THRESHOLD;
  }

  public boolean isReverseLimitTriggered() {
    return piviotEncoder.getAbsolutePosition().getValue() <= PIVIOT_RETRACTED_THRESHOLD;
  }



  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      if (isForwardLimitTriggered()) {
        setState(IntakeState.EXTENDED_NEUTRAL);
      } else { 
        setState(IntakeState.RETRACTED_NEUTRAL);
      }
    }

    piviotDutyCycleControlRequest.withOutput(targetState.getRollerState().getMotorDutyCycle());
    piviotDutyCycleControlRequest.withLimitForwardMotion(isForwardLimitTriggered());
    piviotDutyCycleControlRequest.withLimitReverseMotion(isReverseLimitTriggered());
    pivotLeft.setControl(piviotDutyCycleControlRequest);
    pivotRight.setControl(pivotFollowerRequest);
    rollerMotor.set(targetState.getRollerState().getMotorDutyCycle());
    
  }
}
