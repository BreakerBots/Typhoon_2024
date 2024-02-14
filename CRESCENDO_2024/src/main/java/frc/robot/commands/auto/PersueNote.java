// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import java.util.Optional;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Vision;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
// import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightTarget_Detector;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Intake;

// public class PersueNote extends Command {
//   /** Creates a new PurePursuitIntakeNote. */
//   private Vision vision;
//   private Optional<LimelightTarget_Detector> bestNoteTarget;
//   private PIDController anglePID;
//   private BreakerSwerveVelocityRequest velocityRequest;
//   private Drive drivetrain;
//   private Intake intake;
//   private double persuitVel;
//   public PersueNote() {
//     velocityRequest = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DISABLED, new Translation2d(), 0.02, false, false);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     anglePID.reset();
//     updateVision();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     updateVision();
//     if (bestNoteTarget.isPresent()) {
//       LimelightTarget_Detector tgt = bestNoteTarget.get();
//       ChassisSpeeds speeds = new ChassisSpeeds();
//       speeds.vxMetersPerSecond = persuitVel;
//       if (!(tgt.ty <= -15.0)) {
//         speeds.omegaRadiansPerSecond = anglePID.calculate(tgt.tx, 0.0);
//         velocityRequest.withHeadingCorrectionEnabled(false);
//       } else {
//         velocityRequest.withHeadingCorrectionEnabled(true);
//       }
//       drivetrain.applyRequest(velocityRequest.withChassisSpeeds(speeds));
//     }
//   }

//   private void updateVision() {
//     LimelightTarget_Detector[] detectorTargets = vision.limelight.getLatestResults().targetingResults.targets_Detector;
//     if (detectorTargets.length > 0) {
//       if (bestNoteTarget.isPresent()) {
//         LimelightTarget_Detector prevTgt = bestNoteTarget.get();
//         LimelightTarget_Detector idealTgt = detectorTargets[0];
//         double idealDeltaTx = Math.abs(prevTgt.tx - idealTgt.tx);
//         for (LimelightTarget_Detector tgt : detectorTargets) {
//           double deltaTx = Math.abs(prevTgt.tx - tgt.tx);
//           if (deltaTx < idealDeltaTx) {
//             idealTgt = tgt;
//             idealDeltaTx = deltaTx;
//           }
//         }
//         bestNoteTarget = Optional.of(idealTgt);
//       } else {
//          bestNoteTarget = Optional.of(detectorTargets[0]);
//       }
//     } else {
//       bestNoteTarget = Optional.empty();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return intake.hasNote();
//   }
// }
