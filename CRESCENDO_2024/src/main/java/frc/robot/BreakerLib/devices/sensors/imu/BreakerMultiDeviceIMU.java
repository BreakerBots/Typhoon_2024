package frc.robot.BreakerLib.devices.sensors.imu;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.sensors.accelerometer.BreakerGenericAccelerometer;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGeneric3AxisGyro;

/**
 * Custom IMU class for combining a 3-axis gyro and 3-axis accelerometer into a single
 * IMU object.
 */
public class BreakerMultiDeviceIMU extends BreakerGenericIMU {

    private BreakerGeneric3AxisGyro gyro;
    private BreakerGenericAccelerometer accelerometer;

    /**
     * Creates a BreakerComboIMU.
     * 
     * @param gyro          Gyroscope to use.
     * @param accelerometer Accelerometer to use.
     */
    public BreakerMultiDeviceIMU(BreakerGeneric3AxisGyro gyro, BreakerGenericAccelerometer accelerometer) {
        this.gyro = gyro;
        this.accelerometer = accelerometer;
    }

    @Override
    public double getPitch() {
        return gyro.getPitch();
    }

    @Override
    public double getYaw() {
        return gyro.getYaw();
    }

    @Override
    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return gyro.getPitchRotation2d();
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return gyro.getYawRotation2d();
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return gyro.getRollRotation2d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return gyro.getRotation3d();
    }

    @Override
    public double[] getRawAngles() {
        return gyro.getRawAngles();
    }

    @Override
    public double getRawPitch() {
        return gyro.getRawPitch();
    }

    @Override
    public double getRawYaw() {
        return gyro.getRawYaw();
    }

    @Override
    public double getRawRoll() {
        return gyro.getRawRoll();
    }

    @Override
    public void setPitch(double value) {
        gyro.setPitch(value);
    }

    @Override
    public void setYaw(double value) {
        gyro.setYaw(value);
    }

    @Override
    public void setRoll(double value) {
        gyro.setRoll(value);
    }

    @Override
    public double[] getRawGyroRates() {
        return gyro.getRawGyroRates();
    }

    @Override
    public double getRawPitchRate() {
        return gyro.getRawPitchRate();
    }

    @Override
    public double getRawYawRate() {
        return gyro.getRawYawRate();
    }

    @Override
    public double getRawRollRate() {
        return gyro.getRawRollRate();
    }

    @Override
    public double getPitchRate() {
        return gyro.getPitchRate();
    }

    @Override
    public double getYawRate() {
        return gyro.getYawRate();
    }

    @Override
    public double getRollRate() {
        return gyro.getRollRate();
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return gyro.getRawRotation3d();
    }

    @Override
    public void reset() {
        gyro.reset();

    }

    @Override
    public double[] getRawAccelerometerVals() {
        return accelerometer.getRawAccelerometerVals();
    }

    @Override
    public double getRawAccelX() {
        return accelerometer.getRawAccelX();
    }

    @Override
    public double getRawAccelY() {
        return accelerometer.getRawAccelY();
    }

    @Override
    public double getRawAccelZ() {
        return accelerometer.getRawAccelZ();
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub

    }

    @Override
    public Quaternion getQuaternion() {
        return gyro.getQuaternion();
    }

    @Override
    public void calibrate() {
        gyro.calibrate();
    }

    @Override
    public void close() throws Exception {
        gyro.close();
    }

}
