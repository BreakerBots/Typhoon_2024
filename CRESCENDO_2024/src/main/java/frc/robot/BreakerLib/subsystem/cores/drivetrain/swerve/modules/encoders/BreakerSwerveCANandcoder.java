// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

// /** Add your docs here. */
// public class BreakerSwerveCanAndCoder {
//     // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.reduxrobotics.sensors.canandcoder.CANandcoder;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.Pair;
// import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

// /** Add your docs here. */
// public class BreakerSwerveCANandcoder implements BreakerSwerveAzimuthEncoder {
//     private CANandcoder encoder;
//     public BreakerSwerveCANandcoder(CANandcoder encoder) {
//         this.encoder = encoder;
//     }

//     @Override
//     public double getRelative() {
//         encoder.getPosition;
//         return 0.0;
//        // return BaseStatusSignal.getLatencyCompensatedValue(encoder.getPosition(), encoder.getVelocity()) * 360.0;
//     }

//     @Override
//     public double getAbsolute() {
//         //return MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(encoder.getAbsolutePosition(), encoder.getVelocity()), -0.5, 0.5)  * 360.0;
//         return 0.0;
//     }

//     @Override
//     public Pair<DeviceHealth, String> getFaultData() {
//         //return BreakerPhoenix6Util.checkCANcoderFaultsAndConnection(encoder);
//         return null;
//     }

//     @Override
//     public Class<?> getBaseEncoderType() {
//         return CANcoder.class;
//         return null;
//     }

//     @Override
//     public Object getBaseEncoder() {
//         return encoder;
//         return null;
//     }

//     @Override
//     public void config(boolean invertEncoder, double absoluteOffset) {
//         //BreakerCANCoderFactory.configExistingCANCoder(encoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, absoluteOffset / 360.0, invertEncoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);
//     }

// }
// }
