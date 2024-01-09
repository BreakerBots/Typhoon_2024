// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.BaseTalon;

// import edu.wpi.first.math.Pair;
// import frc.robot.BreakerLib.util.math.BreakerMath;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

// /** CTRE Mag Encoder connected via a Talon SRX. */
// public class BreakerTalonSRXDutyCycleEncoder implements BreakerSwerveAzimuthEncoder {

//     private BaseTalon encoderTalon;
//     private double offset = 0;
//     private int invertSign = 1;

//     public BreakerTalonSRXDutyCycleEncoder(BaseTalon encoderTalon) {
//         this.encoderTalon = encoderTalon;
//         encoderTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
//     }

//     @Override
//     public double getRelative() {
//         return 0;
//     }

//     @Override
//     public double getAbsolute() {
//         return invertSign
//                 * BreakerMath.angleModulus(encoderTalon.getSelectedSensorPosition() * (360.0 / 4096.0) + offset);
//     }

//     @Override
//     public void config(boolean clockwisePositive, double offset) {
//         invertSign = clockwisePositive ? 1 : -1;
//         this.offset = offset;
//     }

//     @Override
//     public Pair<DeviceHealth, String> getFaultData() {
//         // TODO Auto-generated method stub
//         return null;
//     }

//     @Override
//     public Class<?> getBaseEncoderType() {
//         // TODO Auto-generated method stub
//         return encoderTalon.getClass();
//     }

//     @Override
//     public Object getBaseEncoder() {
//         // TODO Auto-generated method stub
//         return encoderTalon;
//     }
// }
