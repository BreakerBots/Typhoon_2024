// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.auto.waypoint;

// import java.util.ArrayList;
// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.LegacyBreakerSwerveDrive.SwerveMovementRefrenceFrame;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
// import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

// /** Add your docs here. */
// public class BreakerSwervePoseWaypointPathFollower extends Command {
//   private BreakerSwerveWaypointFollowerConfig config;
//   private final Timer timer = new Timer();
//   private BreakerHolonomicDriveController driveController;
//   private BreakerPoseWaypointPath waypointPath;
//   private Pose2d prevWp;
//   private ArrayList<Pose2d> waypoints;
//   private boolean stopAtPathEnd;
//   private double totalDistance;
//   private int curTargetWaypointIndex = 0;
//   private int i = 0;
//   private final BreakerSwerveVelocityRequest velocityRequest;

//   /**
//    * Create a BreakerSwerveWaypointFollower with no rotation supplier.
//    * 
//    * @param config Config for the follower.
//    * @param waypointPath Path to follow.
//    */
//   public BreakerSwervePoseWaypointPathFollower(BreakerSwerveWaypointFollowerConfig config, boolean stopAtPathEnd, BreakerPoseWaypointPath waypointPath) {
//     addRequirements(config.getDrivetrain());
//     waypoints = new ArrayList<>();
//     for (Pose2d wp : waypointPath.getWaypoints()) {
//       waypoints.add(wp);
//     }
//     prevWp = config.getOdometer().getOdometryPoseMeters();
//     totalDistance = waypointPath.getTotalPathDistance() + waypoints.get(curTargetWaypointIndex).getTranslation().getDistance(prevWp.getTranslation());
//     this.config = config;
//     this.waypointPath = waypointPath;
//     this.stopAtPathEnd = stopAtPathEnd;
//     driveController = config.getDriveController();
//     velocityRequest = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d());
//   }

//   /** Sets follower to follow new waypoint path.
//    * 
//    * @param newWaypointPath Path to follow.
//    */
//   public void setWaypointPath(BreakerPoseWaypointPath newWaypointPath) {
//     waypointPath = newWaypointPath;
//     waypoints.clear();
//     for (Pose2d wp : waypointPath.getWaypoints()) {
//       waypoints.add(wp);
//     }
//     totalDistance = waypointPath.getTotalPathDistance();
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.stop();
//     timer.reset();
//     curTargetWaypointIndex = 0;
//     BreakerLog.getInstance().logBreakerLibEvent("A new BreakerSwerveWaypointFollower instance has started");
//     prevWp = config.getOdometer().getOdometryPoseMeters();
//     driveController.reset(prevWp);
//     totalDistance += waypoints.get(curTargetWaypointIndex).getTranslation().getDistance(prevWp.getTranslation());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Current values
//     Pose2d curPose = config.getOdometer().getOdometryPoseMeters();
//     ChassisSpeeds targetSpeeds = driveController.calculate(curPose, waypoints.get(curTargetWaypointIndex), waypointPath.getMaxVelocity());
//     // Robot is moved
//     config.getDrivetrain().applyRequest(velocityRequest.withChassisSpeeds(targetSpeeds));
    
//     if (i++%50==0) {
//       System.out.println("\n\n" +targetSpeeds + " | \n" + waypoints + " | \n" + curPose + " \n\n");
//     }
    

//     // Previous waypoint is updated.
//     if (driveController.atTargetPose()) {
//       prevWp = waypoints.get(curTargetWaypointIndex);
//       curTargetWaypointIndex++;
//       System.out.println("WP PASSED");
//     }
//   }

//   /**
//    * @return the internal list that represnets the queue of un-passed waypoints,
//    *         can be modified
//    */
//   public ArrayList<Pose2d> getWaypoints() {
//     return waypoints;
//   }

//   private double getDistanceToWaypoint(Pose2d curPose, Pose2d nextWp) {
//     return curPose.getTranslation().getDistance(nextWp.getTranslation());
//   }

//   private double getTotalRemainingDistance(Pose2d curPose) {
//     double totalDist = getDistanceToWaypoint(curPose, waypoints.get(curTargetWaypointIndex));
//     for (int i = 1; i < waypoints.size(); i++) {
//       totalDist += waypoints.get(i - 1).getTranslation().getDistance(waypoints.get(i).getTranslation());
//     }
//     return totalDist;
//   }

//   /** @return Elapsed path time in seconds. */
//   public double getElapsedTimeSeconds() {
//     return timer.get();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     BreakerLog.getInstance().logBreakerLibEvent("A BreakerSwerveWaypointFollower instance has ended");
//     if (stopAtPathEnd) {
//       config.getDrivetrain().stop();
//     }
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return curTargetWaypointIndex >= waypoints.size();
//   }

// }