// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest.ChassisPercentSpeeds;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.Drive;

public class OrbitNote extends Command {
  /** Creates a new OrbitNote. */
  private Drive drivetrain;
  private PIDController anglePID;
  private Vision vision;
  private BreakerXboxController controller;
  private Optional<LimelightTarget_Detector> bestNoteTarget;
  public OrbitNote(Drive drivetrain, Vision vision, BreakerXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.vision = vision;
    bestNoteTarget = Optional.empty();
    anglePID = new PIDController(0.03, 0, 0.002);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.reset();
    updateVision();
    BreakerLog.recordOutput("IsOrbitingNote", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateVision();
    if (bestNoteTarget.isPresent()) {
      ChassisPercentSpeeds speeds = new ChassisPercentSpeeds();
      speeds.vxPercentOfMax = controller.getLeftThumbstick().getY() * 0.5;
      speeds.vyPercentOfMax = controller.getRightThumbstick().getX() * 0.2;
      BreakerLog.recordOutput("NoteTX", bestNoteTarget.get().tx);
      if (!MathUtil.isNear(0.0, bestNoteTarget.get().tx, 2.0)) {
        speeds.omegaPercentOfMax = anglePID.calculate(bestNoteTarget.get().tx, 0.0);
      }
      drivetrain.applyRequest(new BreakerSwervePercentSpeedRequest(speeds, SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DISABLED, new Translation2d(), 0.02, false, false));
    } else {
      cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    BreakerLog.recordOutput("IsOrbitingNote", false);
  }

  private void updateVision() {
    LimelightTarget_Detector[] detectorTargets = vision.limelight.getLatestResults().targetingResults.targets_Detector;
    if (detectorTargets.length > 0) {
      if (bestNoteTarget.isPresent()) {
        LimelightTarget_Detector prevTgt = bestNoteTarget.get();
        LimelightTarget_Detector idealTgt = detectorTargets[0];
        double idealDeltaTx = Math.abs(prevTgt.tx - idealTgt.tx);
        for (LimelightTarget_Detector tgt : detectorTargets) {
          double deltaTx = Math.abs(prevTgt.tx - tgt.tx);
          if (deltaTx < idealDeltaTx) {
            idealTgt = tgt;
            idealDeltaTx = deltaTx;
          }
        }
        bestNoteTarget = Optional.of(idealTgt);
      } else {
         bestNoteTarget = Optional.of(detectorTargets[0]);
      }
    } else {
      bestNoteTarget = Optional.empty();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bestNoteTarget.isEmpty();
  }
}
