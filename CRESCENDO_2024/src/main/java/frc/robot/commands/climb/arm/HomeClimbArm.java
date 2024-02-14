// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climb.arm;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.ClimbConstants;
// import frc.robot.commands.util.WaitUntilCommandWithConditionSatisfactionDuration;
// import frc.robot.subsystems.ClimbArm;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class HomeClimbArm extends SequentialCommandGroup {
//   /** Creates a new HomeClimbArm. */
//   public HomeClimbArm(ClimbArm climbArm) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new InstantCommand(climbArm::setHomeingSpeed, climbArm),
//       new WaitUntilCommandWithConditionSatisfactionDuration(() -> {return climbArm.getMotorCurrent() >= ClimbConstants.HOMEING_CURRENT;}, 1.5),
//       new InstantCommand(climbArm::home, climbArm)
//     );
//   }
// }
