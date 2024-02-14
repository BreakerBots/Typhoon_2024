// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GeneralConstants;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController.AppliedModifierUnits;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig.BreakerRobotNameConfig;
import frc.robot.subsystems.Drive;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
   private final BreakerPigeon2 imuSys = new BreakerPigeon2(5, GeneralConstants.DRIVE_CANIVORE_NAME);
   private final Drive drivetrainSys = new Drive(imuSys);
  private final BreakerXboxController controllerSys = new BreakerXboxController(0);
  private final BreakerTeleopSwerveDriveController teleopDriveCommand = new BreakerTeleopSwerveDriveController(drivetrainSys, controllerSys);
  // private final Vision visionSys = new Vision(drivetrainSys);

  // private final Intake intakeSys = new Intake();
  // private final Shooter shooterSys = new Shooter();
  // private final PastaRoller pastaRollerSys = new PastaRoller();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriveControls();
    configureBindings();
    configureRobotManager();
  }

  private void configureDriveControls() {
    BreakerLinearizedConstrainedExponential linearMotionTeleopControlCurve = new BreakerLinearizedConstrainedExponential(0.3, 3.0);
    BreakerLinearizedConstrainedExponential angularMotionTeleopControlCurve = new BreakerLinearizedConstrainedExponential(0.0, 3.0);
    controllerSys.configDeadbands(new BreakerGamepadAnalogDeadbandConfig(0.1, 0.1));
    teleopDriveCommand.addSpeedCurves(linearMotionTeleopControlCurve, angularMotionTeleopControlCurve, AppliedModifierUnits.PERCENT_OF_MAX);
    drivetrainSys.setDefaultCommand(teleopDriveCommand);
    
  }

  // public boolean strictHasNote() {
  //   return intakeSys.hasNote() || shooterSys.hasNote() || pastaRollerSys.hasNote();
  // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controllerSys.getButtonA().onTrue(new InstantCommand(drivetrainSys::resetOdometryRotation));
    // controllerSys.getButtonB().toggleOnTrue(new OrbitNote(drivetrainSys, visionSys, controllerSys));
    // controllerSys.getButtonX().and(() -> {return !strictHasNote();}).onTrue(new IntakeFromGroundForShooter(intakeSys, shooterSys));
    // controllerSys.getButtonX().and(intakeSys::hasNote).onTrue(new IntakeToShooterHandoff);
  }

  private void configureRobotManager() {
    BreakerRobotConfig robotConfig = 
    new BreakerRobotConfig(
      new BreakerRobotStartConfig(
        5104, 
        "BreakerBots", 
        new BreakerRobotNameConfig(),
        //  .addRobot(MiscConstants.ROBORIO_SN, "Plop"), 
        2024, 
        "v1 Big Red Edition",
        "Roman Abrahamson, Sebastian Rueda"
        )
      );
      robotConfig.setLogFilePaths("/U/logs", "");
    BreakerRobotManager.setup(drivetrainSys, robotConfig);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    // return AutoBuilder.followPath(path).beforeStarting(() -> {drivetrainSys.setOdometryPosition(path.getPreviewStartingHolonomicPose());});
    return null;
  }
}
