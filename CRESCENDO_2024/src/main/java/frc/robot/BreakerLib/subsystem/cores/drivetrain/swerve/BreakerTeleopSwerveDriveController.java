// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerGenericGamepad;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest.ChassisPercentSpeeds;
import frc.robot.BreakerLib.util.math.functions.BreakerGenericMathFunction;
import frc.robot.BreakerLib.util.math.slewrate.BreakerHolonomicSlewRateLimiter;
import frc.robot.BreakerLib.util.math.slewrate.BreakerHolonomicSlewRateLimiter.UnitlessChassisSpeeds;

/** Controller object for the {@link BreakerLegacySwerveDrive} drivetrain. */
public class BreakerTeleopSwerveDriveController extends Command {

  private BreakerGenericGamepad controller;
  private BreakerSwerveDrive baseDrivetrain;
  private boolean usesSuppliers, usesCurves, usesRateLimiter, turnOverride, forwardOverride, horizontalOverride;

  private BreakerGenericMathFunction linearSpeedCurve, turnSpeedCurve;
  private AppliedModifierUnits speedCurveUnits;

  private BreakerHolonomicSlewRateLimiter slewRateLimiter;
  private AppliedModifierUnits slewRateUnits;

  private DoubleSupplier forwardSpeedPercentSupplier, horizontalSpeedPercentSupplier, turnSpeedPercentSupplier,
      overrideForwardSupplier, overrideHorizontalSupplier, overrideTurnSupplier;
  private AppliedModifierUnits overrideFwdUnits, overrideHorizUnits, overrideTurnUnits;
  private BreakerSwervePercentSpeedRequest percentSpeedRequest;

  // private Timer accelTimer = new Timer();
  // private boolean startedSlew = false;

  private BreakerVector2 lastVelVec = new BreakerVector2();
  
  /**
   * Creates a BreakerSwerveDriveController which only utilizes HID input.
   * 
   * @param baseDrivetrain Swerve drivetrain.
   * @param controller     Xbox controller.
   */
  public BreakerTeleopSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerXboxController controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = false;
    usesRateLimiter = false;
    forwardOverride = false;
    horizontalOverride = false;
    turnOverride = false;
    percentSpeedRequest = new BreakerSwervePercentSpeedRequest(new ChassisPercentSpeeds());
    addRequirements(baseDrivetrain);
  }

  /**
   * Creates a new BreakerSwerveDriveController which uses percent speed values.
   * 
   * @param baseDrivetrain                 The drive train used by this
   *                                       BreakerSwerveDriveController.
   * @param forwardSpeedPercentSupplier    The forward speed percent supplier.
   * @param horizontalSpeedPercentSupplier The horizontal speed percent supplier.
   * @param turnSpeedPercentSupplier       The turn speed percent supplier.
   */
  public BreakerTeleopSwerveDriveController(BreakerSwerveDrive baseDrivetrain, DoubleSupplier forwardSpeedPercentSupplier,
      DoubleSupplier horizontalSpeedPercentSupplier, DoubleSupplier turnSpeedPercentSupplier) {
    this.forwardSpeedPercentSupplier = forwardSpeedPercentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPercentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPercentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = false;
    usesRateLimiter = false;
    forwardOverride = false;
    horizontalOverride = false;
    turnOverride = false;
    percentSpeedRequest = new BreakerSwervePercentSpeedRequest(new ChassisPercentSpeeds());
    addRequirements(baseDrivetrain);
  }

  public BreakerTeleopSwerveDriveController addSlewRateLimiter(BreakerHolonomicSlewRateLimiter slewRateLimiter, AppliedModifierUnits appliedModifierUnits) {
    this.slewRateLimiter = slewRateLimiter;
    usesRateLimiter = true;
    slewRateUnits = appliedModifierUnits;
    return this;
  }

  public BreakerTeleopSwerveDriveController addSpeedCurves(BreakerGenericMathFunction linearSpeedCurve, BreakerGenericMathFunction turnSpeedCurve, AppliedModifierUnits appliedModifierUnits) {
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    usesCurves = true;
    speedCurveUnits = appliedModifierUnits;
    return this;
  }

  public BreakerTeleopSwerveDriveController setSwerveRequest(BreakerSwervePercentSpeedRequest request) {
    this.percentSpeedRequest = request;
    return this;
  }

  /**
   * Overrides turn input with selected percent values.
   * 
   * @param turnSupplier Turn speed percent supplier.
   */
  public void overrideTurnInput(DoubleSupplier turnSupplier, AppliedModifierUnits appliedModifierUnits) {
    turnOverride = true;
    overrideTurnSupplier = turnSupplier;
    overrideTurnUnits = appliedModifierUnits;
  }

  /**
   * Overrides linear inputs with selected percent values.
   * 
   * @param forwardSupplier    Forward speed percent supplier.
   * @param horizontalSupplier Horizontal speed percent supplier.
   */
  public void overrideLinearInput(DoubleSupplier forwardSupplier, DoubleSupplier horizontalSupplier, AppliedModifierUnits appliedModifierUnits) {
    forwardOverride = true;
    horizontalOverride = true;
    overrideHorizontalSupplier = horizontalSupplier;
    overrideForwardSupplier = forwardSupplier;
    overrideFwdUnits = appliedModifierUnits;
    overrideHorizUnits = appliedModifierUnits;
  }

  public void overrideForwardInput(DoubleSupplier forwardSupplier, AppliedModifierUnits appliedModifierUnits) {
    forwardOverride = true;
    overrideForwardSupplier = forwardSupplier;
    overrideFwdUnits = appliedModifierUnits;
  }

  public void overrideHorizontalInput(DoubleSupplier horizontalSupplier, AppliedModifierUnits appliedModifierUnits) {
    horizontalOverride = true;
    overrideHorizontalSupplier = horizontalSupplier;
    overrideHorizUnits = appliedModifierUnits;
  }

  /**
   * Overrides linear and turn inputs with selected percent suppliers.
   * 
   * @param forwardSupplier    Forward speed percent supplier.
   * @param horizontalSupplier Horizontal speed percent supplier.
   * @param turnSupplier       Turn speed percent supplier.
   */
  public void overrideAllInputs(DoubleSupplier forwardSupplier, DoubleSupplier horizontalSupplier,
      DoubleSupplier turnSupplier, AppliedModifierUnits appliedModifierUnits) {
    overrideTurnInput(turnSupplier, appliedModifierUnits);
    overrideLinearInput(horizontalSupplier, forwardSupplier, appliedModifierUnits);
  }

  public void endForwardOverride() {
    forwardOverride = false;
  }

  public void endHorizonalOverride() {
    horizontalOverride = false;
  }


  /** Disables override of linear drive input with percent suppliers. */
  public void endLinearOverride() {
    endForwardOverride();
    endHorizonalOverride();
  }

  /** Disables override of rotation input with a percent supplier. */
  public void endTurnOverride() {
    turnOverride = false;
  }

  /**
   * Disables override of rotation input and linear input with percent suppliers.
   */
  public void endAllOverrides() {
    endLinearOverride();
    endTurnOverride();
  }

  public boolean isForwardInputOverridden() {
      return forwardOverride;
  }

  public boolean isHorizontalInputOverridden() {
      return horizontalOverride;
  }

  /** @return If turn input is being overwritten. */
  public boolean isTurnInputOverridden() {
    return turnOverride;
  }

  public Optional<Pair<BreakerHolonomicSlewRateLimiter,AppliedModifierUnits>> getSlewRateLimiterAndUnits() {
    if (usesRateLimiter) {
      return Optional.of(new Pair<BreakerHolonomicSlewRateLimiter,AppliedModifierUnits>(slewRateLimiter, slewRateUnits));
    }
    return Optional.empty();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    ChassisPercentSpeeds percentSpeeds = new ChassisPercentSpeeds(0.0, 0.0, 0.0);

    double MAX_SPEED_COEFF = 1;
    double MAX_TURN_COEEF = 1;
    if (usesSuppliers) { // If double suppliers are used.
      // Default suppliers are used unless overwritten.
      percentSpeeds.vxPercentOfMax = forwardSpeedPercentSupplier.getAsDouble();
      percentSpeeds.vyPercentOfMax = horizontalSpeedPercentSupplier.getAsDouble();
      percentSpeeds.omegaPercentOfMax = turnSpeedPercentSupplier.getAsDouble();
    } else { // Use controller inputs.
      // Controller inputs are used unless overwritten.
      percentSpeeds.vxPercentOfMax = controller.getLeftThumbstick().getY() * MAX_SPEED_COEFF;
      percentSpeeds.vyPercentOfMax = controller.getLeftThumbstick().getX() * MAX_SPEED_COEFF;
      percentSpeeds.omegaPercentOfMax = controller.getRightThumbstick().getX() * MAX_TURN_COEEF;
    }
      BreakerVector2 interpolLinVelVec = lastVelVec.interpolate(new BreakerVector2(percentSpeeds.vxPercentOfMax, percentSpeeds.vyPercentOfMax), 0.05);
      lastVelVec = interpolLinVelVec;

      percentSpeeds.vxPercentOfMax = interpolLinVelVec.getX();
      percentSpeeds.vyPercentOfMax = interpolLinVelVec.getY();
      


    // Speed curves are applied if overrides are not active.
    if (usesCurves) {
      if (speedCurveUnits == AppliedModifierUnits.PERCENT_OF_MAX) {
        BreakerVector2 vec = new BreakerVector2(percentSpeeds.vyPercentOfMax, percentSpeeds.vxPercentOfMax);
        BreakerVector2 corVec = new BreakerVector2(vec.getVectorRotation(),  linearSpeedCurve.getSignRelativeValueAtX(vec.getMagnitude()));
        percentSpeeds.vxPercentOfMax = corVec.getY();
        percentSpeeds.vyPercentOfMax = corVec.getX();
        percentSpeeds.omegaPercentOfMax = turnSpeedCurve.getSignRelativeValueAtX(percentSpeeds.omegaPercentOfMax);
      } else {
        ChassisSpeeds chassisSpeeds = percentSpeeds.toChassisSpeeds(baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
        BreakerVector2 vec = new BreakerVector2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);
        BreakerVector2 corVec = new BreakerVector2(vec.getVectorRotation(),  linearSpeedCurve.getSignRelativeValueAtX(vec.getMagnitude()));
        chassisSpeeds.vxMetersPerSecond = corVec.getY();
        chassisSpeeds.vyMetersPerSecond = corVec.getX();
        chassisSpeeds.omegaRadiansPerSecond = turnSpeedCurve.getSignRelativeValueAtX(percentSpeeds.omegaPercentOfMax);
        percentSpeeds = new ChassisPercentSpeeds(chassisSpeeds, baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
      }
     
    }

    if (usesRateLimiter) {
      if (slewRateUnits == AppliedModifierUnits.PERCENT_OF_MAX) {
        percentSpeeds = slewRateLimiter.calculate(new UnitlessChassisSpeeds(percentSpeeds)).getChassisPercentSpeeds();
      } else {
        ChassisSpeeds chassisSpeeds = percentSpeeds.toChassisSpeeds(baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
        chassisSpeeds = slewRateLimiter.calculate(new UnitlessChassisSpeeds(chassisSpeeds)).getChassisSpeeds();
        percentSpeeds = new ChassisPercentSpeeds(chassisSpeeds, baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
      }
    }

    if (forwardOverride) {
      if (overrideFwdUnits == AppliedModifierUnits.PERCENT_OF_MAX) {
        percentSpeeds.vxPercentOfMax = overrideForwardSupplier.getAsDouble();
      } else {
        ChassisSpeeds chassisSpeeds = percentSpeeds.toChassisSpeeds(baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
        chassisSpeeds.vxMetersPerSecond = overrideForwardSupplier.getAsDouble();
        percentSpeeds = new ChassisPercentSpeeds(chassisSpeeds, baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
      }
    }

    if (horizontalOverride) {
      if (overrideHorizUnits == AppliedModifierUnits.PERCENT_OF_MAX) {
        percentSpeeds.vyPercentOfMax = overrideHorizontalSupplier.getAsDouble();
      } else {
        ChassisSpeeds chassisSpeeds = percentSpeeds.toChassisSpeeds(baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
        chassisSpeeds.vyMetersPerSecond = overrideHorizontalSupplier.getAsDouble();
        percentSpeeds = new ChassisPercentSpeeds(chassisSpeeds, baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
      }
    }

    if (turnOverride) {
      if (overrideTurnUnits == AppliedModifierUnits.PERCENT_OF_MAX) {
        percentSpeeds.omegaPercentOfMax = overrideTurnSupplier.getAsDouble();
      } else {
        ChassisSpeeds chassisSpeeds = percentSpeeds.toChassisSpeeds(baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
        chassisSpeeds.omegaRadiansPerSecond = overrideTurnSupplier.getAsDouble();
        percentSpeeds = new ChassisPercentSpeeds(chassisSpeeds, baseDrivetrain.getConfig().getMaxLinearVel(), baseDrivetrain.getConfig().getMaxAngleVel());
      }
    }

    baseDrivetrain.applyRequest(percentSpeedRequest.withChassisPercentSpeeds(percentSpeeds));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum AppliedModifierUnits {
    PERCENT_OF_MAX,
    UNIT_PER_SEC
  }
}
