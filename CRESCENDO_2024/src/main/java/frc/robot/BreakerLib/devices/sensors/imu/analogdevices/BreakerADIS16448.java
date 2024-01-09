package frc.robot.BreakerLib.devices.sensors.imu.analogdevices;

import static edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis.kZ;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/**
 * ADIS16448 IMU using BreakerLib interfaces. Calibration is done on
 * construction.
 */
public class BreakerADIS16448 extends BreakerGenericIMU {

    private ADIS16448_IMU imu;
    private Supplier<Double> yawSupplier, pitchSupplier, rollSupplier;
    private Supplier<Double> yawRateSupplier, pitchRateSupplier, rollRateSupplier;

    /**
     * ADIS16448 gyro with Z axis as yaw, MXP SPI port, and calibration time of 512
     * ms.
     */
    public BreakerADIS16448() {
        imu = new ADIS16448_IMU();
        defineAxes(kZ);
        calibrate();
    }

    /**
     * ADIS16448 with custom parameters. All axes are inverted to match the
     * orientations shown in the linked image, regardless of which axis is defined
     * as the yaw.
     * https://wiki.analog.com/_media/first/adis16448_rotation_figure.jpg?w=400&tok=8704ca
     * 
     * @param yawAxis Which axis will be the yaw.
     * @param port    SPI port.
     * @param calTime Calibration time.
     */
    public BreakerADIS16448(IMUAxis yawAxis, SPI.Port port, CalibrationTime calTime) {
        imu = new ADIS16448_IMU(yawAxis, port, calTime);
        defineAxes(yawAxis);
        calibrate();
    }

    /**
     * Defines yaw, pitch, and roll axes based on set yaw axis. Refer to this image:
     * https://wiki.analog.com/_media/first/adis16448_rotation_figure.jpg?w=400&tok=8704ca
     */
    private void defineAxes(IMUAxis yawAxis) {
        switch (yawAxis) {
            case kZ:
                yawSupplier = () -> imu.getGyroAngleZ();
                yawRateSupplier = () -> imu.getGyroRateZ();
                pitchSupplier = () -> imu.getGyroAngleY();
                pitchRateSupplier = () -> imu.getGyroRateY();
                rollSupplier = () -> imu.getGyroAngleX();
                rollRateSupplier = () -> imu.getGyroRateX();
                break;
            case kY:
                yawSupplier = () -> imu.getGyroAngleY();
                yawRateSupplier = () -> imu.getGyroRateY();
                pitchSupplier = () -> -imu.getGyroAngleZ(); // Pitch is inverted.
                pitchRateSupplier = () -> -imu.getGyroRateZ();
                rollSupplier = () -> imu.getGyroAngleX();
                rollRateSupplier = () -> imu.getGyroRateX();
                break;
            case kX:
                yawSupplier = () -> imu.getGyroAngleX();
                yawRateSupplier = () -> imu.getGyroRateX();
                pitchSupplier = () -> imu.getGyroAngleY();
                pitchRateSupplier = () -> imu.getGyroRateY();
                rollSupplier = () -> -imu.getGyroAngleZ(); // Roll is inverted.
                rollRateSupplier = () -> -imu.getGyroRateZ();
                break;
        }
    }

    @Override
    public double getYaw() {
        return BreakerMath.angleModulus(yawSupplier.get());
    }

    @Override
    public double getPitch() {
        return BreakerMath.angleModulus(pitchSupplier.get());
    }

    @Override
    public double getRoll() {
        return BreakerMath.angleModulus(rollSupplier.get());
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
        return yawRateSupplier.get();
    }

    @Override
    public double getPitchRate() {
        return pitchRateSupplier.get();
    }

    @Override
    public double getRollRate() {
        return rollRateSupplier.get();
    }

    @Override
    public double getRawYaw() {
        return yawSupplier.get();
    }

    @Override
    public double getRawPitch() {
        return pitchSupplier.get();
    }

    @Override
    public double getRawRoll() {
        return rollSupplier.get();
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
        // TODO Auto-generated method stub

    }

    @Override
    /** Does nothing. */
    public void setPitch(double value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    /** Does nothing. */
    public void setRoll(double value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    /** Does nothing. */
    public void setYaw(double value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void close() throws Exception {
        imu.close();
        
    }
}
