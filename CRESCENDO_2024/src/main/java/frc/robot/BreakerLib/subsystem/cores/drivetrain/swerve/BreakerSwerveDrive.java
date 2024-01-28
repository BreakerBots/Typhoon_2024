// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveOdometryThread;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest.ChassisPercentSpeeds;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveStopRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLogUtil;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;

/** Add your docs here. */
public class BreakerSwerveDrive extends BreakerGenericDrivetrain /*implements BreakerGenericTestSuiteImplementation<BreakerSwerveDriveTestSuite>*/ {
    private Rotation2d lastSetHeading;
    /**
   * The current {@link SwerveModuleState} each of this drivetrain's swerve
   * modules is set to
   */
  private SwerveModuleState[] targetModuleStates;

  /**
   * Each of the {@link BreakerGenericSwerveModule} instances controlled by this
   * class.
   */
  private BreakerGenericSwerveModule[] swerveModules;

  /**
   * The {@link BreakerGenericGyro} used for this drivetrain's internal odometery.
   */
  private BreakerGenericGyro gyro;

  /**
   * The {@link BreakerGenericOdometer} object this drivetrain uses for its internal
   * odometry.
   */
  private BreakerSwerveOdometryThread odometryThread;

  private BreakerSwerveDriveConfig config;

  private SwerveDriveKinematics kinematics;

  private BreakerSwerveStopRequest stopRequest;

  private Rotation2d fieldRelativeMovementOffset = new Rotation2d();

  private double lowestMaxAttainableModuleWheelSpeed;
  private double furthestModuleDistanceFromRobotCenter;

    public BreakerSwerveDrive(
            BreakerSwerveDriveConfig config,  
            BreakerPathplannerSwerveAutoConfig autoConfig,
            BreakerSwerveOdometryThread odometryThread, 
            BreakerGenericGyro gyro, 
            BreakerGenericSwerveModule... swerveModules) {
        this.config = config;
        this.swerveModules = swerveModules;
        this.gyro = gyro;
        lastSetHeading = getOdometryPoseMeters().getRotation();
        deviceName = "Swerve_Drivetrain";
        targetModuleStates = new SwerveModuleState[swerveModules.length];
        Translation2d[] wheelPositions = new Translation2d[swerveModules.length];
        for (int i = 0; i < targetModuleStates.length; i++) {
          targetModuleStates[i] = new SwerveModuleState();
          wheelPositions[i] = swerveModules[i].getWheelPositionRelativeToRobot();
          double modMaxSpd = swerveModules[i].getMaxAttainableWheelSpeed();
          lowestMaxAttainableModuleWheelSpeed = i < 1 ? modMaxSpd : Math.min(modMaxSpd, lowestMaxAttainableModuleWheelSpeed);
          double modDist = swerveModules[i].getWheelPositionRelativeToRobot().getNorm();
          furthestModuleDistanceFromRobotCenter = i < 1 ? modDist : Math.max(modDist, furthestModuleDistanceFromRobotCenter);
        }
        kinematics = new SwerveDriveKinematics(wheelPositions);
        this.odometryThread = odometryThread;
        odometryThread.start(this);
        stopRequest = new BreakerSwerveStopRequest();
        autoConfig.configureAuto(this);
    }

    public void applyRequest(BreakerSwerveRequest requestToApply) {
        requestToApply.apply(this);
    }

    /**
     * 
     * @param targetChassisSpeeds The desired velocitys of your robot in the X (m/s), Y (m/s), and Rotational (rad/s) axies relative to your chosen refrence frame
     * @param swerveMovementRefrenceFrame The refrence frame from which to apply your robot velocitys
     * @param slowModeValue
     * @param centerOfRotation
     * @param descreteTimestep
     * @param headingCorrectionEnabled
     * @param isOpenLoop
     */
    protected void move(ChassisSpeeds targetChassisSpeeds, SwerveMovementRefrenceFrame swerveMovementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, double descreteTimestep, boolean headingCorrectionEnabled, boolean isOpenLoop) {
        ChassisSpeeds targetVels = BreakerMath.clampChassisSpeeds(targetChassisSpeeds, config.getMaxLinearVel(), config.getMaxAngleVel());
        Rotation2d curAng = getOdometryPoseMeters().getRotation();

        //Convert chasses spees object with a refrence frame defigned by the user to a value that is relative to the robot
        switch(swerveMovementRefrenceFrame) {
            case FIELD_RELATIVE_WITHOUT_OFFSET:
                targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, curAng);
                break;
            case FIELD_RELATIVE_WITH_OFFSET:
                targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
                   curAng.plus(getFieldRelativeMovementOffsetAngle()));
                break;
            case ROBOT_RELATIVE:
            default:
                break;
        }

        //check if heading compensation has been user configed
        Optional<Double> headingCompensationAngularVelDeadband = config.getHeadingCompensationAngularVelDeadband();
        Optional<Double> headingCompensationMinActiveLinearSpeed = config.getHeadingCompensationMinActiveLinearSpeed();
        Optional<PIDController> headingCompensationController = config.getHeadingCompensationController();
        boolean isHeadingCompensationConfiged = headingCompensationAngularVelDeadband.isPresent() && headingCompensationMinActiveLinearSpeed.isPresent() && headingCompensationController.isPresent();
        //if heading comp configed and the requested angular vel is < the ang vel deadband and the linear vel is > than the min active lin speed, lock to last heading that did not satisfy those conditions
        if (isHeadingCompensationConfiged && (Math.abs(targetChassisSpeeds.omegaRadiansPerSecond) < headingCompensationAngularVelDeadband.get() && Math.hypot(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond) > headingCompensationMinActiveLinearSpeed.get())) {
            //if the request calls for heading comp, use a PID controller to lock the robot's heading to the last non-locked heading, overrides user input
            if (headingCorrectionEnabled) {
                targetVels.omegaRadiansPerSecond = MathUtil.clamp(headingCompensationController.get().calculate(curAng.getRadians(), lastSetHeading.getRadians()), -config.getMaxAngleVel(), config.getMaxAngleVel());
            }
        } else {
            lastSetHeading = curAng;
        }

        //ENALED and DISABLED shoud forece this to eval true and false respectivly, DEFAULT lets the request fall back to BreakerGenericDrivetrain's slowModeActive value
        if (slowModeValue == SlowModeValue.ENABLED || (slowModeValue == SlowModeValue.DEFAULT && slowModeActive)) {
            //appls a <1.0 scailar multiplyer ro the target vels
            targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
        }

        //Convererts a vel request ment to be applyed contiuously into vels that can be applyed over the user configed timestep unless < 0.0
        if (descreteTimestep > 0.0) {
          ChassisSpeeds.discretize(targetVels, descreteTimestep);
        }

        //convertes the robot relative chasss speeds into raw swerve module states and applys them with desaturation and opimization
        setModuleStates(false, isOpenLoop, kinematics.toSwerveModuleStates(targetVels, centerOfRotation));
    }


  protected void setModuleStates(boolean applyRawModuleStates, boolean isOpenLoop, SwerveModuleState... targetModuleStates) {
    if (applyRawModuleStates) {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].setModuleTarget(targetModuleStates[i], isOpenLoop);
        this.targetModuleStates[i] = targetModuleStates[i];
      }
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, lowestMaxAttainableModuleWheelSpeed);
      for (int i = 0; i < swerveModules.length; i++) {

        if (Math.abs(targetModuleStates[i].speedMetersPerSecond) < config.getModuleWheelSpeedDeadband()) {

          swerveModules[i].stop();

          this.targetModuleStates[i] = swerveModules[i].getModuleTargetState();

        } else {

          SwerveModuleState optimizedState = SwerveModuleState.optimize(targetModuleStates[i],
              swerveModules[i].getModuleAbsoluteAngle());

          swerveModules[i].setModuleTarget(optimizedState, isOpenLoop);
          this.targetModuleStates[i] = optimizedState;
        }

      }
    }
  }

    public BreakerSwerveDriveConfig getConfig() {
        return config;
    }

    public void setOdometryThread(BreakerSwerveOdometryThread odometer) {
        this.odometryThread = odometer;
        lastSetHeading = getOdometryPoseMeters().getRotation();
    }

    public BreakerSwerveOdometryThread getOdometryThread() {
        return odometryThread;
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometryThread.setOdometryPosition(newPose);
        lastSetHeading = newPose.getRotation();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            lastSetHeading = getOdometryPoseMeters().getRotation();
        }
    }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /** @return States of swerve modules. */
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }
  
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] moduleStates = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModulePosition();
    }
    return moduleStates;
  }

  public void setFieldRelativeMovementOffsetAngle(Rotation2d fieldRelativeMovementOffset) {
    this.fieldRelativeMovementOffset = fieldRelativeMovementOffset;
  }

  public Rotation2d getFieldRelativeMovementOffsetAngle() {
    return fieldRelativeMovementOffset;
  }
  
  public void resetSwerveModuleDriveDistances() {
    for (BreakerGenericSwerveModule mod : swerveModules) {
      mod.resetModuleDriveEncoderPosition();
    }
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return odometryThread.getOdometryPoseMeters();
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return odometryThread.getMovementState();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return odometryThread.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return odometryThread.getFieldRelativeChassisSpeeds();
  }

  public void setStatusUpdatePeriod(double period) {
    for(BreakerGenericSwerveModule mod : swerveModules) {
      mod.setStatusUpdatePeriod(period);
    }
    gyro.setStatusUpdatePeriod(period);
  }

  @Override
  public void runSelfTest() {
    faultStr = "";
    health = DeviceHealth.NOMINAL;
    for (BreakerGenericSwerveModule module : swerveModules) {
      module.runSelfTest();
      if (module.hasFault()) {
        faultStr += " " + module.getDeviceName() + ": " + module.getFaults() + " ";
        health = health != DeviceHealth.INOPERABLE ? module.getHealth() : health;
      }
    }
  }

  // @Override
  // public BreakerSwerveDriveTestSuite getTestSuite() {
  //   return new BreakerSwerveDriveTestSuite(this, swerveModules);
  // }


  @Override
  public BreakerGenericGyro getBaseGyro() {
    return gyro;
  }

  @Override
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    for (BreakerGenericSwerveModule module : swerveModules) {
      if (RobotState.isEnabled()) {
        module.setDriveMotorBrakeMode(isEnabled);
        module.setTurnMotorBrakeMode(true);
      } else {
        module.setModuleBrakeMode(isEnabled);
      }
    }
  }

  public SwerveModuleState[] getTargetModuleStates() {
    return targetModuleStates;
  }

  public double getLowestMaxAttainableModuleWheelSpeed() {
      return lowestMaxAttainableModuleWheelSpeed;
  }


  @Override
  public void stop() {
    applyRequest(stopRequest);
  }

  @Override
  public String toString() {
    return String.format("BreakerSwerveDrive(Health: %s, Movement_State: %s, Swerve_Modules: %s)", health.toString(),
        odometryThread.getMovementState().toString(), Arrays.toString(swerveModules));
  }

  public ChassisSpeeds getTargetChassisSpeeds() {
      return kinematics.toChassisSpeeds(targetModuleStates);
  }

  public double getFurthestModuleDistanceFromRobotCenter() {
    return furthestModuleDistanceFromRobotCenter;
  }

  @Override
  public void toLog(LogTable table) {
    table.put("DeviceHealth", getHealth().toString());
    table.put("RealModuleStates", getSwerveModuleStates());
    table.put("TargetModuleStates", targetModuleStates);
    odometryThread.toLog(table.getSubtable("Odometry"));
    LogTable moduleTable = table.getSubtable("Modules");
    for (BreakerGenericSwerveModule module: swerveModules) {
      module.toLog(moduleTable.getSubtable(module.getDeviceName()));
    }
  }

  public enum SwerveMovementRefrenceFrame {
    FIELD_RELATIVE_WITH_OFFSET,
    FIELD_RELATIVE_WITHOUT_OFFSET,
    ROBOT_RELATIVE
  }

  public interface BreakerSwerveRequest {
    public abstract void apply(BreakerSwerveDrive drivetrain);
    public default void applyChassisSpeeds(BreakerSwerveDrive drivetrain, ChassisSpeeds targetChassisSpeeds, SwerveMovementRefrenceFrame swerveMovementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, double descreteTimestep, boolean headingCorrectionEnabled, boolean isOpenLoop) {
      drivetrain.move(targetChassisSpeeds, swerveMovementRefrenceFrame, slowModeValue, centerOfRotation, descreteTimestep, headingCorrectionEnabled, isOpenLoop);
    }

    public default void applyModuleStates(BreakerSwerveDrive drivetrain, boolean applyRawModuleStates, boolean isOpenLoop, SwerveModuleState... targetModuleStates) {
      drivetrain.setModuleStates(applyRawModuleStates, isOpenLoop, targetModuleStates);
    }
  }

  

  public static interface BreakerPathplannerSwerveAutoConfig {
  
    public abstract void configureAuto(BreakerSwerveDrive drivetrain);
    public abstract BreakerSwerveRequest getAutoRequest();

    public static class BreakerPathplannerStandardSwerveAutoConfig implements BreakerPathplannerSwerveAutoConfig {
      private double dt;
      private ReplanningConfig replanningConfig;
      private PIDConstants linearPIDConstants, rotationalPIDConstants;
      private boolean mirrorPathToAlliance;
      private final BreakerSwerveVelocityRequest autoRequest;
      public BreakerPathplannerStandardSwerveAutoConfig(PIDConstants linearPIDConstants, PIDConstants rotationalPIDConstants) {
        this(linearPIDConstants, rotationalPIDConstants, new ReplanningConfig(), 0.02, true);
      }

      public BreakerPathplannerStandardSwerveAutoConfig(PIDConstants linearPIDConstants, PIDConstants rotationalPIDConstants, ReplanningConfig replanningConfig, double dt, boolean mirrorPathToAlliance) {
        this.dt = dt;
        this.replanningConfig = replanningConfig;
        this.rotationalPIDConstants = rotationalPIDConstants;
        this.linearPIDConstants = linearPIDConstants;
        this.mirrorPathToAlliance = mirrorPathToAlliance;
        autoRequest = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DISABLED, new Translation2d(), dt, false, false);
      }

      public BreakerSwerveVelocityRequest getAutoRequest() {
          return autoRequest;
      }

      private boolean mirrorPathSupplyer() {
        Optional<Alliance> ally = DriverStation.getAlliance(); 
        if (ally.isPresent() && mirrorPathToAlliance) {
          return ally.get() == Alliance.Red;
        }
        return false;
      }

      @Override
      public void configureAuto(BreakerSwerveDrive drivetrain) {
        AutoBuilder.configureHolonomic(
          drivetrain::getOdometryPoseMeters,
          drivetrain::setOdometryPosition, 
          drivetrain::getRobotRelativeChassisSpeeds, 
          (ChassisSpeeds speeds) -> drivetrain.applyRequest(autoRequest.withChassisSpeeds(speeds)),
          new HolonomicPathFollowerConfig(
            linearPIDConstants, 
            rotationalPIDConstants, 
            drivetrain.getLowestMaxAttainableModuleWheelSpeed(), 
            drivetrain.getFurthestModuleDistanceFromRobotCenter(), 
            replanningConfig, 
            dt
          ),
          this::mirrorPathSupplyer,
          drivetrain
        );
      }
    }
  }
}
