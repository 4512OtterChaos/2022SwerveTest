package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveConstants {

    private static final boolean isReal = RobotBase.isReal();
    public static final int kPigeonID = 0;

    // Inversions
    public static final boolean kInvertGyro = false;
    public static final boolean kInvertDrive = false;
    public static final boolean kInvertSteer = false;
    public static final boolean kInvertCancoder = false;
    // Physical properties
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kTrackLength = Units.inchesToMeters(18.5);
    
    public static final double kMaxLinearSpeed = Units.feetToMeters(14);
    public static final double kMaxAngularSpeed = Units.degreesToRadians(720);
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter*Math.PI;
    public static final double kDriveGearRatio = 6.75; // 6.75:1
    public static final double kSteerGearRatio = 12.8; // 12.8:1

    public enum Module {
        FL(1, 0, 1, 0, 96.855, kTrackLength/2, kTrackWidth/2), // Front left
        FR(2, 2, 3, 1, -118.565, kTrackLength/2, -kTrackWidth/2),
        BL(3, 4, 5, 2, -122.344, -kTrackLength/2, kTrackWidth/2),
        BR(4, 6, 7, 3, 175.078, -kTrackLength/2, -kTrackWidth/2);
        
        public final int moduleNum;
        public final int driveMotorID;
        public final int steerMotorID;
        public final int cancoderID;
        public final double angleOffset;
        public final Translation2d centerOffset;
        private Module(int moduleNum, int driveMotorID, int steerMotorID, int cancoderID, double angleOffset, double xOffset, double yOffset){
            this.moduleNum = moduleNum;
            this.driveMotorID = driveMotorID;
            this.steerMotorID = steerMotorID;
            this.cancoderID = cancoderID;
            this.angleOffset = angleOffset;
            centerOffset = new Translation2d(xOffset, yOffset);
        }
    }

    // Current limits
    public static final int kDriveContinuousCurrentLimit = 40;
    public static final int kDrivePeakCurrentLimit = 65;
    public static final double kDrivePeakCurrentDuration = 0.1;
    public static final int kSteerContinuousCurrentLimit = 25;
    public static final int kSteerPeakCurrentLimit = 40;
    public static final double kSteerPeakCurrentDuration = 0.1;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    public static final int kAllowableSteeringError = 80;

    // Feedforward
    // Linear drive feed forward
    public static final SimpleMotorFeedforward kDriveFF = isReal ?
        new SimpleMotorFeedforward( // real
            0.2, // Voltage to break static friction
            2.25, // Volts per meter per second
            0.17 // Volts per meter per second squared
        )
        :
        new SimpleMotorFeedforward( // sim
            0, // Voltage to break static friction -- we do not use kS with this method of simulation
            2.5, // Volts per meter per second
            0.4 // Volts per meter per second squared -- lower kA will give snappier control
        );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerFF = isReal ?
        new SimpleMotorFeedforward( // real
            0.55, // Voltage to break static friction
            0.2, // Volts per radian per second
            0.0056 // Volts per radian per second squared
        )
        :
        new SimpleMotorFeedforward( // sim
            0, // Voltage to break static friction
            0.15, // Volts per radian per second
            0.002 // Volts per radian per second squared
        );

    // PID
    public static final double kDriveKP = 0.0;
    public static final double kDriveKI = 0;
    public static final double kDriveKD = 0;

    //public static final double kSteerKP = isReal ? 0.75 : 0.1;
    //public static final double kSteerKI = isReal ? 0 : 0;
    //public static final double kSteerKD = isReal ? 20 : 0;

    public static final double kSteerKP = isReal ? 2 : 0.1;
    public static final double kSteerKI = isReal ? 0 : 0;
    public static final double kSteerKD = isReal ? 0 : 0;

    public static final double kSteerVelocity = Units.degreesToRadians(1440);
    public static final double kSteerAcceleration = Units.degreesToRadians(3240);


    // The configurations applied to swerve CTRE devices
    public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    public static final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    public static final PigeonIMUConfiguration pigeonConfig = new PigeonIMUConfiguration();

    static {
        driveConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        driveConfig.slot0.kP = kDriveKP;
        driveConfig.slot0.kI = kDriveKI;
        driveConfig.slot0.kD = kDriveKD;
        driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kDriveContinuousCurrentLimit,
            kDrivePeakCurrentLimit,
            kDrivePeakCurrentDuration
        );
        driveConfig.voltageCompSaturation = kVoltageSaturation;
        driveConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
        driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        driveConfig.velocityMeasurementWindow = 32;

        steerConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        steerConfig.slot0.kP = kSteerKP;
        steerConfig.slot0.kI = kSteerKI;
        steerConfig.slot0.kD = kSteerKD;
        steerConfig.slot0.allowableClosedloopError = kAllowableSteeringError;
        steerConfig.neutralDeadband = isReal ? 0.05 : 0.001;
        steerConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kSteerContinuousCurrentLimit,
            kSteerPeakCurrentLimit,
            kSteerPeakCurrentDuration
        );
        steerConfig.voltageCompSaturation = kVoltageSaturation;
        steerConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
        steerConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        steerConfig.velocityMeasurementWindow = 32;

        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cancoderConfig.sensorDirection = kInvertCancoder;
    }
}
