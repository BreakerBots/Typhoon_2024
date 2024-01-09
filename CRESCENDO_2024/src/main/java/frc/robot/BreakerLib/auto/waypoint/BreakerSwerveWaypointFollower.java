// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.LegacyBreakerSwerveDrive.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

/** Add your docs here. */
public class BreakerSwerveWaypointFollower extends Command {
  private BreakerSwerveWaypointFollowerConfig config;
  private final Timer timer = new Timer();
  private BreakerHolonomicDriveController driveController;
  private BreakerWaypointPath waypointPath;
  private Supplier<Rotation2d> rotationSupplier;
  private Translation2d prevWp;
  private ArrayList<Translation2d> waypoints;
  private boolean stopAtPathEnd;
  private double totalDistance;
  private int curTargetWaypointIndex = 0;
  private int i = 0;
  private final BreakerSwerveVelocityRequest velocityRequest;

  /**
   * Create a BreakerSwerveWaypointFollower with no rotation supplier.
   * 
   * @param config Config for the follower.
   * @param waypointPath Path to follow.
   */
  public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, boolean stopAtPathEnd, BreakerWaypointPath waypointPath) {
    addRequirements(config.getDrivetrain());
    waypoints = new ArrayList<>();
    for (Translation2d wp : waypointPath.getWaypoints()) {
      waypoints.add(wp);
    }
    prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
    totalDistance = waypointPath.getTotalPathDistance() + waypoints.get(curTargetWaypointIndex).getDistance(prevWp);
    this.config = config;
    this.waypointPath = waypointPath;
    this.stopAtPathEnd = stopAtPathEnd;
    rotationSupplier = () -> (config.getOdometer().getOdometryPoseMeters().getRotation());
    driveController = config.getDriveController();
    velocityRequest = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d());
  }

  /**
   * Create a BreakerSwerveWaypointFollower with rotation supplier.
   * 
   * @param config Config for the follower.
   * @param waypointPath Path to follow.
   * @param rotationSupplier Supplier of swerve rotation. Useful for CV target tracking et al.
   */
  public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, boolean stopAtPathEnd, BreakerWaypointPath waypointPath,
      Supplier<Rotation2d> rotationSupplier) {
    addRequirements(config.getDrivetrain());
    waypoints = new ArrayList<>();
    for (Translation2d wp : waypointPath.getWaypoints()) {
      waypoints.add(wp);
    }

    totalDistance = waypointPath.getTotalPathDistance();
    this.config = config;
    this.waypointPath = waypointPath;
    this.rotationSupplier = rotationSupplier;
    this.stopAtPathEnd = stopAtPathEnd;
    driveController = config.getDriveController();
    velocityRequest = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d());
  }

  /** Sets follower to follow new waypoint path.
   * 
   * @param newWaypointPath Path to follow.
   */
  public void setWaypointPath(BreakerWaypointPath newWaypointPath) {
    waypointPath = newWaypointPath;
    waypoints.clear();
    for (Translation2d wp : waypointPath.getWaypoints()) {
      waypoints.add(wp);
    }
    totalDistance = waypointPath.getTotalPathDistance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    BreakerLog.getInstance().logBreakerLibEvent("A new BreakerSwerveWaypointFollower instance has started");
    prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
    totalDistance += waypoints.get(curTargetWaypointIndex).getDistance(prevWp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Current values
    Pose2d curPose = config.getOdometer().getOdometryPoseMeters();
    ChassisSpeeds targetSpeeds = driveController.calculate(curPose, new Pose2d(waypoints.get(curTargetWaypointIndex), rotationSupplier.get()), waypointPath.getMaxVelocity());
    // Robot is moved
    config.getDrivetrain().applyRequest(velocityRequest.withChassisSpeeds(targetSpeeds));
    
    if (i++%50==0) {
      System.out.println("\n\n" +targetSpeeds + " | \n" + waypoints + " | \n" + curPose + " \n\n");
    }
    

    // Previous waypoint is updated.
    if (driveController.atTargetPose()) {
      prevWp = waypoints.get(curTargetWaypointIndex);
      curTargetWaypointIndex++;
      System.out.println("WP PASSED");
    }
  }

  /**
   * @return the internal list that represnets the queue of un-passed waypoints,
   *         can be modified
   */
  public ArrayList<Translation2d> getWaypoints() {
    return waypoints;
  }

  private double getDistanceToWaypoint(Pose2d curPose, Translation2d nextWp) {
    return curPose.getTranslation().getDistance(nextWp);
  }

  private double getTotalRemainingDistance(Pose2d curPose) {
    double totalDist = getDistanceToWaypoint(curPose, waypoints.get(curTargetWaypointIndex));
    for (int i = 1; i < waypoints.size(); i++) {
      totalDist += waypoints.get(i - 1).getDistance(waypoints.get(i));
    }
    return totalDist;
  }

  /** @return Elapsed path time in seconds. */
  public double getElapsedTimeSeconds() {
    return timer.get();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BreakerLog.getInstance().logBreakerLibEvent("A BreakerSwerveWaypointFollower instance has ended");
    if (stopAtPathEnd) {
      config.getDrivetrain().stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return curTargetWaypointIndex >= waypoints.size();
  }

}
