// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.PathlessAutoTest;
import frc.robot.commands.auto.PersueNote;
import frc.robot.commands.handoffs.HandoffFromIntakeToShooter;
import frc.robot.commands.intake.ExtakeNote;
import frc.robot.commands.intake.IntakeFromGroundForShooter;
import frc.robot.subsystems.ClimbArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.PastaRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final BreakerPigeon2 imuSys = new BreakerPigeon2(5, GeneralConstants.DRIVE_CANIVORE_NAME);
  private static final Drive drivetrainSys = new Drive(imuSys);
  public static final BreakerXboxController controllerSys = new BreakerXboxController(0);
  private final BreakerTeleopSwerveDriveController teleopDriveCommand = new BreakerTeleopSwerveDriveController(drivetrainSys, controllerSys);
  private final Vision visionSys = new Vision(drivetrainSys);

  private final Intake intakeSys = new Intake();
  private final Shooter shooterSys = new Shooter(Constants.ShooterConstants.MANUAL_SPEAKER_SHOT_FIREING_SOLUTION_SUPPLIER);
  private final PastaRoller pastaRollerSys = new PastaRoller();

  public static final ClimbArm leftClimbSys = new ClimbArm(50, Constants.GeneralConstants.DRIVE_CANIVORE_NAME, true);
  public static final ClimbArm rigtClimbSys = new ClimbArm(51, Constants.GeneralConstants.DRIVE_CANIVORE_NAME, true);

  public static final ShooterTarget SPEAKER_TARGET = new ShooterTarget(drivetrainSys, Constants.FieldConstants.BLUE_SPEAKER_AIM_POINT, Constants.ShooterConstants.FIREING_MAP);

  
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
    //teleopDriveCommand.addSlewRateLimiter(new BreakerHolonomicSlewRateLimiter(0.3, -5.0, 3.0, -5.0, new UnitlessChassisSpeeds(0.0, 0.0, 0.0)), AppliedModifierUnits.PERCENT_OF_MAX);
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
    //controllerSys.getButtonB().toggleOnTrue(new OrbitNote(drivetrainSys, visionSys, controllerSys));
    //controllerSys.getButtonB().onTrue(new IntakeFromGroundForShooter(intakeSys, shooterSys));
    //  controllerSys.getButtonX().onTrue(intakeSys.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false));


    //controllerSys.getButtonY().onTrue(new HandoffToPastaRollerTest(intakeSys, pastaRollerSys));

    controllerSys.getLeftBumper().onTrue(intakeSys.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false));
    controllerSys.getRightBumper().onTrue(intakeSys.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false));

    //controllerSys.getButtonB().toggleOnTrue(new OrbitNote(drivetrainSys, visionSys, controllerSys));
    controllerSys.getButtonB().onTrue(new ExtakeNote(intakeSys));
    controllerSys.getButtonY().onTrue(intakeSys.setStateCommand(IntakeState.EXTENDED_INTAKEING, true).andThen(new PersueNote(visionSys, intakeSys, drivetrainSys), intakeSys.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true)));

    controllerSys.getButtonX()
      .and(intakeSys::hasNote)
      .and(()-> {return !shooterSys.hasNote();})
      .and(() -> {return intakeSys.getState() != IntakeState.EXTENDED_EXTAKEING;})
      .onTrue(new HandoffFromIntakeToShooter(shooterSys, intakeSys, false));
    controllerSys.getButtonX()
      .debounce(0.1, DebounceType.kBoth)
      .and(()-> {return !intakeSys.hasNote();})
      .and(shooterSys::hasNote)
      .and(() -> {return intakeSys.getState() != IntakeState.EXTENDED_EXTAKEING;})
      .onTrue(new StationaryShootFromAnywhere(shooterSys, drivetrainSys));
    controllerSys.getButtonX()
      .and(()-> {return !intakeSys.hasNote();})
      .and(()-> {return !shooterSys.hasNote();})
      .and(() -> {return shooterSys.getState() != ShooterState.TRACK_TARGET;})
      .onTrue(new IntakeFromGroundForShooter(intakeSys, shooterSys));

  }

  private void registerNamedCommands() {
    HashMap<String, Command> namedCommands = new HashMap<>();
    namedCommands.put("IntakeFromGroundForShooter", new IntakeFromGroundForShooter(intakeSys, shooterSys));
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
        "v1.5",
        "Roman Abrahamson, Sebastian Rueda, and Aiden Copiaco"
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
    return new PathlessAutoTest(shooterSys, drivetrainSys, intakeSys, visionSys);
  }
}
