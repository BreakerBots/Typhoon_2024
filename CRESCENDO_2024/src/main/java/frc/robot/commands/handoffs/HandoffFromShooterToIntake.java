// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.handoffs;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.SuperstructureState;
// import frc.robot.commands.SetSuperstructureState;
// import frc.robot.commands.util.WaitUntilCommndWithFallingEdgeDelayAndTimeout;
// import frc.robot.subsystems.Superstructure;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class HandoffFromShooterToIntake extends SequentialCommandGroup {
//   /** Creates a new HandoffFromShooterToIntake. */
//   public HandoffFromShooterToIntake(Superstructure superstructure) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_EXTENDED_HOLD, true),
//       new SetSuperstructureState(superstructure, SuperstructureState.SHOOTER_TO_INTAKE_HANDOFF, false),
//       new WaitUntilCommndWithFallingEdgeDelayAndTimeout(superstructure::intakeHasNote, 0.35, 2.0),
//       new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_EXTENDED_HOLD, false)
//     );
//   }
// }
