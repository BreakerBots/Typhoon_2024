// No!

package frc.robot.BreakerLib.devices.sensors.imu.analogdevices;

import static edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kX;
import static edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kY;
import static edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kZ;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/**
 * ADIS16470 IMU using BreakerLib interfaces. Calibration is done on
 * construction. Only Z-axis yaw is supported.
 */
public class BreakerADIS16470 extends BreakerGenericIMU {

    private ADIS16470_IMU imu;

    /**
     * ADIS16470 gyro with Z axis as yaw, CS0 SPI port, and calibration time of 4
     * ms.
     */
    public BreakerADIS16470() {
        imu = new ADIS16470_IMU();
        calibrate();
    }

    /**
     * ADIS16470 with custom parameters. Refer to the following image for orientation.
     * https://wiki.analog.com/_media/first/adis16470_rotation_figure.jpg?w=250&tok=187ece
     * 
     * @param port    SPI port.
     * @param calTime Calibration time.
     */
    public BreakerADIS16470(SPI.Port port, CalibrationTime calTime) {
        imu = new ADIS16470_IMU(kZ, port, calTime);
        calibrate();
    }

    @Override
    public double getYaw() {
        return BreakerMath.angleModulus(getRawYaw());
    }

    @Override
    public double getPitch() {
        return BreakerMath.angleModulus(getRawPitch());
    }

    @Override
    public double getRoll() {
        return BreakerMath.angleModulus(getRawRoll());
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRoll());
    }

    @Override
    public double getYawRate() {
        imu.setYawAxis(kZ);
        double rate = imu.getRate();
        return rate;
    }

    @Override
    public double getPitchRate() {
        imu.setYawAxis(kY);
        double rate = imu.getRate();
        return rate;
    }

    @Override
    public double getRollRate() {
        imu.setYawAxis(kX);
        double rate = imu.getRate();
        return rate;
    }

    /** Does nothing. */
    public void setYaw(double value) {
    }

    /** Does nothing. */
    public void setPitch(double value) {
    }

    /** Does nothing. */
    public void setRoll(double value) {

    }

    @Override
    public double getRawYaw() {
        imu.setYawAxis(kZ);
        double angle = imu.getAngle();
        return angle;
    }

    @Override
    public double getRawPitch() {
        imu.setYawAxis(kY);
        double angle = imu.getAngle();
        return angle;
    }

    @Override
    public double getRawRoll() {
        imu.setYawAxis(kX);
        double angle = imu.getAngle();
        return angle;
    }

    @Override
    public double getRawYawRate() {
        return getYawRate();
    }

    @Override
    public double getRawPitchRate() {
        return getPitchRate();
    }

    @Override
    public double getRawRollRate() {
        return getRollRate();
    }

    @Override
    public double[] getRawAngles() {
        return new double[] {getRawYaw(), getRawPitch(), getRawRoll()};
    }

    @Override
    public double[] getRawGyroRates() {
        return new double[] {getRawYawRate(), getRawPitchRate(), getRawRollRate()};
    }

    @Override
    public Quaternion getQuaternion() {
        return getRotation3d().getQuaternion();
    }

    @Override
    public Rotation3d getRotation3d() {
        return new Rotation3d(getRoll(), getPitch(), getYaw());
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return new Rotation3d(getRawRoll(), getRawPitch(), getRawYaw());
    }

    @Override
    public void calibrate() {
        imu.calibrate();
    }

    @Override
    public void reset() {
        imu.reset();
    }

    @Override
    public double[] getRawAccelerometerVals() {
        return new double[] {getRawAccelX(), getRawAccelY(), getRawAccelZ()};
    }

    @Override
    public double getRawAccelX() {
        return BreakerUnits.metersPerSecondSquaredToGs(imu.getAccelX());
    }

    @Override
    public double getRawAccelY() {
        return BreakerUnits.metersPerSecondSquaredToGs(imu.getAccelY());
    }

    @Override
    public double getRawAccelZ() {
        return BreakerUnits.metersPerSecondSquaredToGs(imu.getAccelZ());
    }

    /** Does nothing. */
    public void setRange(Range range) {
        
    }

    @Override
    public void runSelfTest() {
       

    }

    @Override
    public void close() throws Exception {
       imu.close();
        
    }
}
