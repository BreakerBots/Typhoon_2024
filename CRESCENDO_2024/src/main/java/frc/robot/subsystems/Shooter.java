// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static frc.robot.Constants.ShooterConstants.STOW_ANGLE;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.ShooterTarget;
// import frc.robot.ShooterTarget.FireingSolution;
// import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

// public class Shooter extends SubsystemBase {
//   /** Creates a new Shooter. */
//   private TalonFX piviotMotor;
//   private TalonFX flywheelLeft, flywheelRight;
//   private WPI_TalonSRX hopper;
//   private ShooterTarget target;
//   private ShooterState state;
//   private DoubleSupplier flywheelVelSup;
//   private DoubleSupplier flywheelAccelSup;
//   private DoubleSupplier pivotPosSup;
//   private DoubleSupplier pivotVelSup;
//   private VelocityVoltage flywheelVelRequest;
//   private Follower flywheelFollowRequest;
//   private MotionMagicVoltage pivotMotionMagicRequest;
//   private FireingSolution latestFireingSolution;
//   private BreakerBeamBreak beamBreak;
//   public Shooter(ShooterTarget defaultTarget) {
//     target = defaultTarget;
//   }

//   private void configFlywheel() {

//   }

//   private void configPivot() {

//   }

//   public void setState(ShooterState state) {

//   }

//   public void setActiveTarget(ShooterTarget target) {
//     this.target = target;
//   }

//   public FireingSolution getActiveTargetFireingSolution() {
//       return latestFireingSolution;
//   }

//   public boolean hasNote() {
//     return beamBreak.isBroken();
//   }

//   public boolean isAtGoal() {
//     return isAtAngleGoal() && isAtFlywheelGoal();
//   }

//   public boolean isAtAngleGoal() {
//     return MathUtil.isNear(pivotMotionMagicRequest.Position, pivotPosSup.getAsDouble(), Units.degreesToRotations(0.5)) && MathUtil.isNear(0.0, pivotVelSup.getAsDouble(),  Units.degreesToRotations(0.1));
//   }

//   public boolean isAtFlywheelGoal() {
//     return MathUtil.isNear(pivotMotionMagicRequest.Position, flywheelVelSup.getAsDouble(), 5.0 / 60.0) && MathUtil.isNear(0.0, flywheelAccelSup.getAsDouble(), 0.5 / 60) ;
//   }


//   public static enum ShooterHopperState {
//     FORWARD(0.75),
//     REVERSE(-0.4),
//     NEUTRAL(0.0);
//     private double dutyCycle;
//     private ShooterHopperState(double dutyCycle) {
//       this.dutyCycle = dutyCycle;
//     }

//     public double getDutyCycle() {
//         return dutyCycle;
//     }
//   }

//   public static enum ShooterState {
//     TRACK_TARGET(ShooterHopperState.NEUTRAL),
//     SHOOT_TO_TARGET(ShooterHopperState.FORWARD),
//     INTAKE_TO_SHOOTER_HANDOFF(ShooterHopperState.FORWARD),
//     SHOOTER_TO_INTAKE_HANDOFF(ShooterHopperState.REVERSE),
//     STOW(ShooterHopperState.NEUTRAL);

//     private ShooterHopperState hopperState;
//     private ShooterState(ShooterHopperState hopperState) {
//       this.hopperState = hopperState;
//     }

//     public ShooterHopperState getHopperState() {
//         return hopperState;
//     }
//   }

//   public double getFlywheelVel() {
//     return flywheelVelSup.getAsDouble();
//   }

//   public double getFlywheelAccel() {
//     return flywheelAccelSup.getAsDouble();
//   }

//   public Rotation2d getShooterAngle() {
//     return Rotation2d.fromRotations(pivotPosSup.getAsDouble());
//   }

//   private void pushControlRequests(double hopperDutyCycle, double piviotPos, double flywheelVel) {
//     hopper.set(hopperDutyCycle);
//     piviotMotor.setControl(pivotMotionMagicRequest.withPosition(piviotPos));
//     flywheelLeft.setControl(flywheelVelRequest.withVelocity(flywheelVel));
//     flywheelRight.setControl(flywheelFollowRequest);
//   }
  
//   @Override
//   public void periodic() {
//     latestFireingSolution = target.getFireingSolution();
//     switch (state) {
//       case SHOOT_TO_TARGET:
//       case TRACK_TARGET:
//         pushControlRequests(state.getHopperState().getDutyCycle(), latestFireingSolution.fireingVec().getVectorRotation().getRotations(), latestFireingSolution.fireingVec().getMagnitude());
//         break;
//       case STOW:
//       case INTAKE_TO_SHOOTER_HANDOFF:
//       case SHOOTER_TO_INTAKE_HANDOFF:
//       default:
//         pushControlRequests(state.getHopperState().getDutyCycle(), STOW_ANGLE.getRotations(), latestFireingSolution.fireingVec().getMagnitude());
//         break;

//     }
//   }
// }
