// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors.imu.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp.TimestampSource;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.sensors.BreakerGenericMagnetometer;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/* CTRE Pigeon 2 implementing the Breaker device interface and Breaker IMU interface,  */
public class BreakerPigeon2 extends BreakerGenericIMU implements BreakerGenericMagnetometer {
  private Pigeon2 pigeon;
  private BreakerPigeon2PeriodicIO periodicIO;

  /** Creates a new PigeonIMU 2 object. */
  public BreakerPigeon2(int deviceID) {
    pigeon = new Pigeon2(deviceID);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
    periodicIO = this.new BreakerPigeon2PeriodicIO();
  }

  /** Creates a new PigeonIMU 2 object. */
  public BreakerPigeon2(int deviceID, String busName) {
    pigeon = new Pigeon2(deviceID, busName);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
    periodicIO = this.new BreakerPigeon2PeriodicIO();

  }

  // public Pigeon2SimState getSimState() {
  //   return pigeon.getSimState();
  // }

  public Pigeon2Configurator getConfigurator() {
    return pigeon.getConfigurator();
  }

  @Override
  public double getPitch() {
    return periodicIO.getPitch();
  }

  @Override
  public double getYaw() {
    return periodicIO.getYaw();
  }

  @Override
  public double getRoll() {
    return periodicIO.getRoll();
  }

  @Override
  public Rotation2d getPitchRotation2d() {
    return Rotation2d.fromDegrees(getPitch());
  }

  @Override
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  @Override
  public Rotation2d getRollRotation2d() {
    return Rotation2d.fromDegrees(getRoll());
  }

  @Override
  public Rotation3d getRotation3d() {
    return new Rotation3d(getQuaternion());
  }

  @Override
  public double[] getRawAngles() {
    return new double[]{periodicIO.getRawYaw(), periodicIO.getRawPitch(), periodicIO.getRawRoll()};
  }

  @Override
  public double getRawYaw() {
    return getRawAngles()[0];
  }

  @Override
  public double getRawPitch() {
    return getRawAngles()[1];
  }

  @Override
  public double getRawRoll() {
    return getRawAngles()[2];
  }

  /** Does nothing. */
  @Override
  public void setPitch(double value) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setYaw(double value) {
    pigeon.setYaw(0);
  }

  /** Does nothing. */
  @Override
  public void setRoll(double value) {
  }

  /** Sets yaw to 0 */
  @Override
  public void reset() {
    pigeon.setYaw(0);
  }

  public double[] getRawGyroRates() {
    return new double[] {periodicIO.getRawYawRate(), periodicIO.getRawPitchRate(), periodicIO.getRawRollRate()};
  }

  @Override
  public double getRawPitchRate() {
    return getRawGyroRates()[0];
  }

  @Override
  public double getRawRollRate() {
    return getRawGyroRates()[1];
  }

  @Override
  public double getRawYawRate() {
    return getRawGyroRates()[2];
  }

  @Override
  public double getPitchRate() {
    return getRawPitchRate();
  }

  @Override
  public double getYawRate() {
    return getRawYawRate();
  }

  @Override
  public double getRollRate() {
    return getRawRollRate();
  }

  @Override
  public BreakerVector3 getRawAccelerometerVals() {
    return new BreakerVector3(getRawAccelX(), getRawAccelY(), getRawAccelZ());
  }

  @Override
  /** @return Unbiased accelerometer x-value in G. */
  public double getRawAccelX() {
    return pigeon.getAccelerationX().getValue();
  }

  @Override
  /** @return Unbiased accelerometer y-value in G. */
  public double getRawAccelY() {
    return pigeon.getAccelerationY().getValue();
  }

  @Override
  /** @return Unbiased accelerometer z-value in G. */
  public double getRawAccelZ() {
    return pigeon.getAccelerationZ().getValue();
  }

  /** @return Pigeon's runtime in seconds (max of 255) */
  public int getPigeonUpTime() {
    return pigeon.getUpTime().getValue().intValue();
  }

  public BreakerVector3 getGravityVector() {
    return new BreakerVector3(pigeon.getGravityVectorX().getValue(), pigeon.getGravityVectorY().getValue(), pigeon.getGravityVectorZ().getValue());
  }

  public double[] getBiasedAccelerometerVals() {
    BreakerVector3 vec = new BreakerVector3(getRawAccelX(), getRawAccelY(), getRawAccelZ()).minus(getGravityVector());
    return new double[] {vec.getX(), vec.getY(), vec.getZ()};
  }

  /** @return Biased accelerometer x-value in G. */
  public double getBaisedAccelX() {
    return getBiasedAccelerometerVals()[0];
  }

  /** @return Biased accelerometer y-value in G. */
  public double getBaisedAccelY() {
    return getBiasedAccelerometerVals()[1];
  }

  /** @return Biased accelerometer z-value in G. */
  public double getBiasedAccelZ() {
    return getBiasedAccelerometerVals()[2];
  }

  @Override
  public Rotation3d getRawRotation3d() {
    return new Rotation3d(Math.toRadians(getRawAngles()[2]), Math.toRadians(getRawAngles()[1]),
        Math.toRadians(getRawAngles()[0]));
  }

  @Override
  public void runSelfTest() {
    faultStr = "";
    health = DeviceHealth.NOMINAL;
    Pair<DeviceHealth, String> pair = BreakerPhoenix6Util.checkPigeon2FaultsAndConnection(pigeon);
    if (pair.getFirst() != DeviceHealth.NOMINAL) {
      health = pair.getFirst();
      faultStr = pair.getSecond();
    }
  }

  @Override
  public BreakerVector3 getRawFieldStrengths() {
    return new BreakerVector3(pigeon.getRawMagneticFieldX().getValue(), pigeon.getRawMagneticFieldY().getValue(), pigeon.getRawMagneticFieldZ().getValue());
  }

  @Override
  public BreakerVector3 getBiasedFieldStrengths() {
    return new BreakerVector3(pigeon.getMagneticFieldX().getValue(), pigeon.getMagneticFieldY().getValue(), pigeon.getMagneticFieldZ().getValue());

  }

  @Override
  /** not supported by phoenix 6 */
  public double getCompassFieldStrength() {
    return getRawFieldStrengths().getMagnitude();
  }

  @Override
  /** not supported by phoenix 6 */
  public Rotation2d getCompassHeading() {
    return getBiasedFieldStrengths().getVectorRotation().toRotation2d();
  }

  @Override
  /** not supported by phoenix 6 */
  public double getRawCompassHeading() {
    return getRawFieldStrengths().getVectorRotation().toRotation2d().getDegrees();
  }

  @Override
  public Quaternion getQuaternion() {
    return new Quaternion(pigeon.getQuatW().getValue(), pigeon.getQuatX().getValue(), pigeon.getQuatY().getValue(), pigeon.getQuatZ().getValue());
  }

  private class BreakerPigeon2PeriodicIO {
      private StatusSignal<Double> rawYaw, rawPitch, rawRoll;
      private StatusSignal<Double> rawYawRate, rawPitchRate, rawRollRate;
      public BreakerPigeon2PeriodicIO() {
        rawYaw = pigeon.getYaw();
        rawPitch = pigeon.getPitch();
        rawRoll = pigeon.getRoll();
        rawYawRate = pigeon.getAngularVelocityZ();
        rawPitchRate = pigeon.getAngularVelocityX();
        rawRollRate = pigeon.getAngularVelocityY();
      }

      public double getYaw() {
        return BreakerMath.angleModulus(BaseStatusSignal.getLatencyCompensatedValue(rawYaw.refresh(), rawYawRate.refresh()));
      }

      public double getRoll() {
        return BreakerMath.angleModulus(BaseStatusSignal.getLatencyCompensatedValue(rawRoll.refresh(), rawRollRate.refresh()));
      }

      public double getPitch() {
        return BreakerMath.angleModulus(BaseStatusSignal.getLatencyCompensatedValue(rawPitch.refresh(), rawPitchRate.refresh()));
      }

      public double getRawYaw() {
          return rawYaw.refresh().getValue();
      }

      public double getRawRoll() {
          return rawRoll.refresh().getValue();
      }

      public double getRawPitch() {
          return rawPitch.refresh().getValue();
      }

      public double getRawPitchRate() {
          return rawPitchRate.refresh().getValue();
      }

      public double getRawRollRate() {
          return rawRollRate.refresh().getValue();
      }

      public double getRawYawRate() {
          return rawYawRate.refresh().getValue();
      }
  }

}
