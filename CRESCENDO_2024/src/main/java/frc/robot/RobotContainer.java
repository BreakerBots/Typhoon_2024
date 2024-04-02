// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GeneralConstants;
import frc.robot.BreakerLib.auto.BreakerAutoPath;
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
import frc.robot.commands.AllignToAmp;
import frc.robot.commands.OrbitNote;
import frc.robot.commands.ScoreInAmp;
import frc.robot.commands.StationaryShootFromAnywhere;
import frc.robot.commands.auto.paths.CenterShoot4InWing;
import frc.robot.commands.auto.paths.CenterThenGoDeepShoot3;
import frc.robot.commands.auto.paths.FUNauto;
import frc.robot.commands.auto.paths.FiveNoteAuto;
import frc.robot.commands.auto.paths.LeaveShootOneSource;
import frc.robot.commands.auto.paths.SourceShoot3GoToCenter;
import frc.robot.commands.auto.paths.ThreeNoteA;
import frc.robot.commands.auto.paths.ThreeNoteAgainstSpeaker;
import frc.robot.commands.handoffs.HandoffFromIntakeToShooter;
import frc.robot.commands.handoffs.HandoffFromShooterToIntake;
import frc.robot.commands.intake.ExtakeNote;
import frc.robot.commands.intake.IntakeFromGround;
import frc.robot.commands.intake.IntakeFromGroundForPastaRoller;
import frc.robot.commands.intake.IntakeFromGroundForShooter;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.shooter.ShootManualAllign;
import frc.robot.commands.shooter.ShootManualFloat;
import frc.robot.commands.shooter.SpoolShooterForSpeakerShot;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.ClimbArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDState;
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
  public static BreakerXboxController controllerSys = new BreakerXboxController(0);
  private final BreakerTeleopSwerveDriveController teleopDriveCommand = new BreakerTeleopSwerveDriveController(drivetrainSys, controllerSys);
  private static final Trigger globalOverride = controllerSys.getStartButton() ;
  private final Vision visionSys = new Vision(drivetrainSys, true);

 
  private final Shooter shooterSys = new Shooter(RobotContainer.SPEAKER_TARGET::getFireingSolution);
  private final Intake intakeSys = new Intake();
  private final AmpBar ampBarSys = new AmpBar();
  private final LED led = new LED(shooterSys, intakeSys);

  


  public static final ClimbArm leftClimbSys = new ClimbArm(50, "rio", true);
  public static final ClimbArm rigtClimbSys = new ClimbArm(51, "rio", true);

  public static final ShooterTarget SPEAKER_TARGET = new ShooterTarget(drivetrainSys, Constants.FieldConstants.BLUE_SPEAKER_AIM_POINT, Constants.ShooterConstants.FIREING_MAP);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDriveControls();
    configureBindings();
    configureRobotManager();
    registerNamedCommands();
  }

  private void configureDriveControls() {
    BreakerLinearizedConstrainedExponential linearMotionTeleopControlCurve = new BreakerLinearizedConstrainedExponential(0.075, 3.0);
    BreakerLinearizedConstrainedExponential angularMotionTeleopControlCurve = new BreakerLinearizedConstrainedExponential(0.0, 3.0);
    controllerSys.configDeadbands(new BreakerGamepadAnalogDeadbandConfig(0.1, 0.1));
    teleopDriveCommand.addSpeedCurves(linearMotionTeleopControlCurve, angularMotionTeleopControlCurve, AppliedModifierUnits.PERCENT_OF_MAX);
    //teleopDriveCommand.addSlewRateLimiter(new BreakerHolonomicSlewRateLimiter(0.5, -100.0, 10.0, -10.0, new UnitlessChassisSpeeds(0.0, 0.0, 0.0)), AppliedModifierUnits.PERCENT_OF_MAX);
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

    controllerSys.getButtonB()
      .and(() -> intakeSys.getState().getPivotState() != IntakePivotState.RETRACTED)
      .onTrue(new ExtakeNote(intakeSys, shooterSys));
      // .onTrue(new AllignToAmp(drivetrainSys));

    controllerSys.getLeftBumper()
      .and(() -> intakeSys.getState() != IntakeState.RETRACTED_EXTAKEING)
      .onTrue(new StowIntake(intakeSys, ampBarSys, shooterSys));

    controllerSys.getButtonA()
      .debounce(0.1, DebounceType.kBoth)
      .and(()-> {return !intakeSys.hasNote();})
      .and(shooterSys::hasNote)
      .and(() -> {return intakeSys.getState() != IntakeState.EXTENDED_EXTAKEING;})
      .onTrue(new SequentialCommandGroup(
        led.returnToRestState(), // shooting is typically the end of a state
        new ShootManualFloat(shooterSys),
        led.returnToRestState()
       ));
     
    //   .and(() -> !intakeSys.hasNote())
    //   .and(() -> !shooterSys.hasNote())
    //   .and(() -> intakeSys.getState() != IntakeState.EXTENDED_INTAKEING)
    //   .onTrue(new IntakeFromGround(intakeSys));
    // controllerSys.getButtonA()
    //   .and(() -> !intakeSys.hasNote())
    //   .and(() -> !shooterSys.hasNote())
    //   .and(() -> intakeSys.getState() == IntakeState.EXTENDED_INTAKEING)
    //   .toggleOnTrue(
    //     new OrbitNote(drivetrainSys, visionSys, controllerSys)
    //     .onlyWhile(() -> !(intakeSys.hasNote() || shooterSys.hasNote()) && (intakeSys.getState() == IntakeState.EXTENDED_EXTAKEING))
    //   );

    controllerSys.getButtonY()
      .and(() -> shooterSys.hasNote())
      .and(() -> !intakeSys.hasNote())
      .onTrue(new HandoffFromShooterToIntake(shooterSys, intakeSys, true));
    controllerSys.getButtonY()
      .debounce(0.1, DebounceType.kBoth)
      .and(() -> intakeSys.hasNote())
      .and(() -> !shooterSys.hasNote())
      .and(() -> intakeSys.getState().getPivotState() == IntakePivotState.RETRACTED)
      .onTrue(new ScoreInAmp(intakeSys, ampBarSys).andThen(led.returnToRestState()));
    controllerSys.getButtonY()
      .and(() -> !intakeSys.hasNote())
      .and(() -> !shooterSys.hasNote())
      .onTrue(led.returnToRestState().andThen(new IntakeFromGroundForPastaRoller(intakeSys), led.returnToRestState()));

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
      .onTrue(new SequentialCommandGroup(
        led.returnToRestState(), // shooting is typically the end of a state
        new ConditionalCommand(
          new ShootManualAllign(shooterSys),  
          new StationaryShootFromAnywhere(shooterSys, drivetrainSys), 
          globalOverride),
        led.returnToRestState()
       ));

    controllerSys.getButtonX()
      .and(()-> {return !intakeSys.hasNote();})
      .and(()-> {return !shooterSys.hasNote();})
      .and(() -> {return shooterSys.getState() != ShooterState.TRACK_TARGET;})
      .onTrue(new SequentialCommandGroup(
        led.setStateCommand(LEDState.INTAKING_FOR_SHOOTER),
        new IntakeFromGroundForShooter(intakeSys, shooterSys, led),
        led.returnToRestState()
        ));


    controllerSys.getRightThumbstick().getJoystickButton()
      .and(() -> shooterSys.hasNote())
      .and(() -> !intakeSys.hasNote())
      .and(() -> intakeSys.getState() != IntakeState.EXTENDED_EXTAKEING)
      .toggleOnTrue(new SpoolShooterForSpeakerShot(shooterSys, true));

  }

  public static Trigger getGlobalOverride() {
    return globalOverride;
  }

  private void registerNamedCommands() {
    HashMap<String, Command> namedCommands = new HashMap<>();
    namedCommands.put("IntakeFromGroundForShooter", new IntakeFromGroundForShooter(intakeSys, shooterSys, led));
    namedCommands.put("SpoolShooter", new SpoolShooterForSpeakerShot(shooterSys, false));
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
      robotConfig.setAutoPaths(
        new BreakerAutoPath("AmpSideShoot3", new ThreeNoteAgainstSpeaker(shooterSys, drivetrainSys, intakeSys, visionSys)),
        new BreakerAutoPath("CenterShoot3", new CenterShoot4InWing(shooterSys, drivetrainSys, intakeSys, visionSys)),
        new BreakerAutoPath("CenterThenGoDeepShoot3", new CenterThenGoDeepShoot3(shooterSys, drivetrainSys, intakeSys, visionSys)),
        new BreakerAutoPath("SourceShoot3GoToCenter", new SourceShoot3GoToCenter(shooterSys, drivetrainSys, intakeSys, visionSys)),
        new BreakerAutoPath("FiveishNoteAuto", new FiveNoteAuto(shooterSys, drivetrainSys, intakeSys, visionSys)),
        new BreakerAutoPath("LeaveShootOneSourceSide", new LeaveShootOneSource(drivetrainSys, shooterSys)),
        new BreakerAutoPath("ShootFromAnywhereAndStop", new WaitCommand(2.0).andThen(new StationaryShootFromAnywhere(shooterSys, drivetrainSys)))
      );
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

    return BreakerRobotManager.getSelectedAutoPath();
  }
}