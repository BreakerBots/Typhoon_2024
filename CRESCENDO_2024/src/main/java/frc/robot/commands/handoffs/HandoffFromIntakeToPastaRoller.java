// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.handoffs;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.SuperstructureState;
// import frc.robot.commands.SetSuperstructureState;
// import frc.robot.subsystems.Superstructure;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class HandoffFromIntakeToPastaRoller extends SequentialCommandGroup {
//   /** Creates a new HandoffFromIntakeToPastaRoller. */
//   public HandoffFromIntakeToPastaRoller(Superstructure superstructure) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new SetSuperstructureState(superstructure, SuperstructureState.STOW, true),
//       new SetSuperstructureState(superstructure, SuperstructureState.INTAKE_TO_PASTA_ROLLER_HANDOFF, false),
//       new WaitUntilCommand(superstructure::pastaRollerHasNote)
//     );
//   }
// }
