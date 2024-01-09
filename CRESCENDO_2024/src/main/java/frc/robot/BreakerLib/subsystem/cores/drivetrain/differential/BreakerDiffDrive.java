// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveState;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups.BreakerDiffDriveMotorGroup;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

public class BreakerDiffDrive extends BreakerGenericDrivetrain {
  /** Creates a new BreakerDiffDrive. */
  protected BreakerDiffDriveMotorGroup leftMotorGroup, rightMotorGroup;
  private BreakerDiffDriveConfig config;
  private BreakerGenericOdometer odometer;
  private BreakerGenericGyro gyro;

  public BreakerDiffDrive(BreakerDiffDriveConfig config, BreakerDiffDriveOdometryConfig odometryConfig, BreakerGenericGyro gyro, BreakerDiffDriveMotorGroup leftMotorGroup, BreakerDiffDriveMotorGroup rightMotorGroup) {
    this.config = config;
    this.leftMotorGroup = leftMotorGroup;
    this.rightMotorGroup = rightMotorGroup;
    this.gyro = gyro;
    odometer = odometryConfig.getOdometer(this);
  }

  public BreakerDiffDrive(BreakerDiffDriveConfig config, BreakerGenericGyro gyro, BreakerDiffDriveMotorGroup leftMotorGroup, BreakerDiffDriveMotorGroup rightMotorGroup) {
    this(config, new BreakerDiffDriveOdometryConfig(), gyro, leftMotorGroup, rightMotorGroup);
  }

  public void setRawWheelSpeeds(double leftSpeed, double rightSpeed) {
    leftMotorGroup.set(leftSpeed);
    rightMotorGroup.set(rightSpeed);
  }

  public void setRawWheelSpeeds(WheelSpeeds wheelSpeeds) {
    setRawWheelSpeeds(wheelSpeeds.left, wheelSpeeds.right);
  }

  public void setRawWheelVoltages(DifferentialDriveWheelVoltages wheelVoltages) {
    setRawWheelVoltages(wheelVoltages.left, wheelVoltages.right);
  }

  public void setRawWheelVoltages(double leftVoltage, double rightVoltage) {
    leftMotorGroup.setVoltage(leftVoltage);
    rightMotorGroup.setVoltage(rightVoltage);
  }

  public void arcadeDrive(double xSpeed, double zRotation, BreakerDiffDriveMovementPrefrences movementPrefrences) {
    setRawWheelSpeeds(arcadeDriveIK(xSpeed, zRotation, movementPrefrences));
  }

  protected WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation, BreakerDiffDriveMovementPrefrences movementPrefrences) {
    if (movementPrefrences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPrefrences.getSlowModeValue() == SlowModeValue.DEFAULT && slowModeActive)) {
      xSpeed *= config.getSlowModeForwardMultiplier();
      zRotation *= config.getSlowModeForwardMultiplier();
    }

    if (movementPrefrences.getInputDeadband() != 0.0) {
      xSpeed = MathUtil.applyDeadband(xSpeed, movementPrefrences.getInputDeadband());
      zRotation = MathUtil.applyDeadband(zRotation, movementPrefrences.getInputDeadband());
    }

    return DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, movementPrefrences.getSquareInputsEnabled());
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace, BreakerDiffDriveMovementPrefrences movementPrefrences) {
    setRawWheelSpeeds(curvatureDriveIK(xSpeed, zRotation, allowTurnInPlace, movementPrefrences));
  }

  protected WheelSpeeds curvatureDriveIK(double xSpeed, double zRotation, boolean allowTurnInPlace, BreakerDiffDriveMovementPrefrences movementPrefrences) {
    if (movementPrefrences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPrefrences.getSlowModeValue() == SlowModeValue.DEFAULT && slowModeActive)) {
      xSpeed *= config.getSlowModeForwardMultiplier();
      zRotation *= config.getSlowModeForwardMultiplier();
    }

    if (movementPrefrences.getSquareInputsEnabled()) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    if (movementPrefrences.getInputDeadband() != 0.0) {
      xSpeed = MathUtil.applyDeadband(xSpeed, movementPrefrences.getInputDeadband());
      zRotation = MathUtil.applyDeadband(zRotation, movementPrefrences.getInputDeadband());
    }

    return DifferentialDrive.curvatureDriveIK(xSpeed, zRotation, allowTurnInPlace);
  }

  public void tankDrive(double leftSpeed, double rightSpeed, BreakerDiffDriveMovementPrefrences movementPrefrences) {
    setRawWheelSpeeds(tankDriveIK(leftSpeed, rightSpeed, movementPrefrences));
  }

  protected WheelSpeeds tankDriveIK(double leftSpeed, double rightSpeed, BreakerDiffDriveMovementPrefrences movementPrefrences) {
    if (movementPrefrences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPrefrences.getSlowModeValue() == SlowModeValue.DEFAULT && slowModeActive)) {
      leftSpeed *= config.getSlowModeForwardMultiplier();
      rightSpeed *= config.getSlowModeForwardMultiplier();
    }
    if (movementPrefrences.getInputDeadband() != 0.0) {
      leftSpeed = MathUtil.applyDeadband(leftSpeed, movementPrefrences.getInputDeadband());
      rightSpeed = MathUtil.applyDeadband(rightSpeed, movementPrefrences.getInputDeadband());
    }
    return DifferentialDrive.tankDriveIK(leftSpeed, rightSpeed, movementPrefrences.getSquareInputsEnabled());
  }


  public DifferentialDriveKinematics getKinematics() {
    return config.getKinematics();
  }

  public double getLeftDriveWheelSpeed() {
    return leftMotorGroup.getEncoderVelocity() / config.getEncoderRotationsPerMeter();
  }

  public double getLeftDriveWheelDistance() {
    return leftMotorGroup.getEncoderVelocity() / config.getEncoderRotationsPerMeter();
  }

  public double getRightDriveWheelSpeed() {
    return leftMotorGroup.getEncoderVelocity() / config.getEncoderRotationsPerMeter();
  }

  public double getRightDriveWheelDistance() {
    return rightMotorGroup.getEncoderVelocity() / config.getEncoderRotationsPerMeter();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftDriveWheelSpeed(), getRightDriveWheelSpeed());
  }

  public DifferentialDriveWheelVoltages getWheelVoltages() {
    return new DifferentialDriveWheelVoltages(leftMotorGroup.getOutputVoltage() * RobotController.getBatteryVoltage(), rightMotorGroup.getOutputVoltage() * RobotController.getBatteryVoltage());
  }

  public BreakerDiffDriveState getDriveState() {
    return new BreakerDiffDriveState(getWheelSpeeds(), getLeftDriveWheelDistance(), getLeftDriveWheelDistance());
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    odometer.setOdometryPosition(newPose);
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return odometer.getOdometryPoseMeters();
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return odometer.getMovementState();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return odometer.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return odometer.getFieldRelativeChassisSpeeds();
  }

  @Override
  public void runSelfTest() {
    health = DeviceHealth.NOMINAL;
    faultStr = "";
    leftMotorGroup.runSelfTest();
    rightMotorGroup.runSelfTest();
    if (leftMotorGroup.hasFault()) {
      faultStr += leftMotorGroup.getFaults();
      if (health != DeviceHealth.INOPERABLE) {
        health = leftMotorGroup.getHealth();
      }
    }
    if (rightMotorGroup.hasFault()) {
      faultStr += rightMotorGroup.getFaults();
      if (health != DeviceHealth.INOPERABLE) {
        health = rightMotorGroup.getHealth();
      }
    }
  }

  @Override
  public BreakerGenericGyro getBaseGyro() {
    return gyro;
  }

  @Override
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    leftMotorGroup.setBrakeMode(isEnabled);
    rightMotorGroup.setBrakeMode(isEnabled);
  }

  public BreakerDiffDriveConfig getConfig() {
      return config;
  }

  @Override
  public void stop() {
    setRawWheelSpeeds(new WheelSpeeds());
  }

  public static class BreakerDiffDriveMovementPrefrences {
    protected final boolean squareInputsEnabled;
    protected final double inputDeadband;
    protected final SlowModeValue slowModeValue;
    public BreakerDiffDriveMovementPrefrences(boolean squareInputsEnabled, double inputDeadband, SlowModeValue slowModeValue) {
      this.squareInputsEnabled = squareInputsEnabled;
      this.slowModeValue = slowModeValue;
      this.inputDeadband = inputDeadband;
    }

    public BreakerDiffDriveMovementPrefrences withSquareInputsEnabled(boolean isEnabled) {
      return new BreakerDiffDriveMovementPrefrences(squareInputsEnabled, inputDeadband, slowModeValue);
    }

    public BreakerDiffDriveMovementPrefrences withSlowModeValue(SlowModeValue slowModeValue) {
      return new BreakerDiffDriveMovementPrefrences(squareInputsEnabled, inputDeadband, slowModeValue);
    }

    public BreakerDiffDriveMovementPrefrences withInputDeadband(double deadband) {
      return new BreakerDiffDriveMovementPrefrences(squareInputsEnabled, deadband, slowModeValue);
    }

    public SlowModeValue getSlowModeValue() {
        return slowModeValue;
    }

    public boolean getSquareInputsEnabled() {
        return squareInputsEnabled;
    }

    public double getInputDeadband() {
        return inputDeadband;
    }
  }

  public static class BreakerDiffDriveOdometryConfig {
    private BreakerGenericVisionOdometer vision;
    private Pose2d initalPose;
    private double[] stateStanderdDeveation, visionStanderdDeveation;
    private boolean usePoseEstimator;

    public BreakerDiffDriveOdometryConfig() {
       this(new Pose2d());
    }

    public BreakerDiffDriveOdometryConfig(Pose2d initalPose) {
        this.initalPose = initalPose;
        usePoseEstimator = false;
    }

    public BreakerDiffDriveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        Pose2d initalPose,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
       this.initalPose = initalPose;
       this.vision = vision;
       this.stateStanderdDeveation = stateStanderdDeveation;
       this.visionStanderdDeveation = visionStanderdDeveation;
       usePoseEstimator = true;
    }

    public BreakerDiffDriveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
        this(vision, new Pose2d(), stateStanderdDeveation, visionStanderdDeveation);
    }

    public BreakerGenericOdometer getOdometer(BreakerDiffDrive drivetrain) {
        if (usePoseEstimator) {
            return new BreakerDiffDriveFusedVisionPoseEstimator(drivetrain, vision, initalPose, visionStanderdDeveation, stateStanderdDeveation);
        }
        return new BreakerDiffDriveOdometer(drivetrain, initalPose);
    }
  }


}
