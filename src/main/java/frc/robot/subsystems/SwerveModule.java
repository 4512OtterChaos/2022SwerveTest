package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.constants.SwerveConstants;
import frc.robot.util.FalconUtil;


public class SwerveModule {

    private final SwerveConstants.Module moduleConstants;
    private SwerveModuleState desiredState = new SwerveModuleState();
    private double lastTargetTotalAngle = 0;
    private boolean hasSetup = false;

    // Hardware
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;
    private final WPI_CANCoder steerEncoder;

    // Linear drive feed forward
    public final SimpleMotorFeedforward driveFF = kDriveFF;
    // Steer feed forward
    public final SimpleMotorFeedforward steerFF = kSteerFF;
    public final ProfiledPIDController steerController = new ProfiledPIDController( 
        kSteerKP, kSteerKI, kSteerKD, 
        new TrapezoidProfile.Constraints(kSteerVelocity, kSteerAcceleration)
    );

    public SwerveModule(SwerveConstants.Module moduleConstants){
        this.moduleConstants = moduleConstants;

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        steerMotor = new WPI_TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new WPI_CANCoder(moduleConstants.cancoderID);

        setupDriveMotor();
        setupCancoder();
        setupSteerMotor();
        hasSetup = true;
        steerController.enableContinuousInput(-Math.PI, Math.PI);
        // Simulation
        driveMotorSim = driveMotor.getSimCollection();
        steerMotorSim = steerMotor.getSimCollection();
        steerEncoderSim = steerEncoder.getSimCollection();
    }

    private void setupDriveMotor(){
        if(!hasSetup){
            driveMotor.configAllSettings(driveConfig);
        }
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(kInvertDrive);
        if(RobotBase.isReal()) FalconUtil.configStatusFrames(driveMotor);
    }
    private void setupCancoder(){
        steerEncoder.configAllSettings(cancoderConfig);
        steerEncoder.configMagnetOffset(moduleConstants.angleOffset, 50);
    }
    private void setupSteerMotor(){
        if(!hasSetup){
            steerMotor.configAllSettings(steerConfig);
        }
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setInverted(kInvertSteer);
        resetToAbsolute();
        if(RobotBase.isReal()) FalconUtil.configStatusFrames(steerMotor);
    }

    public void periodic(){
        // check if the motors had an oopsie, reapply settings
        if(driveMotor.hasResetOccurred()){
            setupDriveMotor();
        }
        if(steerMotor.hasResetOccurred()){
            setupSteerMotor();
        }
    }

    /**
     * Reset the steering motor integrated encoder to the position of the steering cancoder.
     * We want to use the integrated encoder for control, but need the absolute cancoder for determining our startup rotation.
     */
    public void resetToAbsolute(){
        double absolutePosition = FalconUtil.degreesToPosition(getAbsoluteHeading().getDegrees(), kSteerGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
        lastTargetTotalAngle = FalconUtil.positionToRadians(absolutePosition, kSteerGearRatio);
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean steerInPlace){
        Rotation2d currentRotation = getIntegratedHeading();
        SwerveModuleState lastDesiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);
        
        
        // our desired angle in [-pi, pi]
        double targetConstrainedAngle = desiredState.angle.getRadians();
        // our total current angle. This is not constrained to [-pi, pi]
        double currentTotalAngle = currentRotation.getRadians();
        // our current angle in [-pi, pi]
        double currentConstrainedAngle = MathUtil.angleModulus(currentTotalAngle);
        double angleDelta = MathUtil.angleModulus(targetConstrainedAngle - currentConstrainedAngle);
        // convert our constrained target to the closest "total" angle near our current total
        double targetTotalAngle = currentTotalAngle + angleDelta;

        // if the module is not driving, maintain last angle setpoint
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            targetTotalAngle = lastTargetTotalAngle;
            this.desiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        }
        else{
            lastTargetTotalAngle = targetTotalAngle;
            this.desiredState = desiredState;
        }

        // convert our target radians to falcon position units
        double angleNative = FalconUtil.radiansToPosition(targetTotalAngle, kSteerGearRatio);
        // perform onboard PID to steer the module to the target angle
        /*steerMotor.set(
            ControlMode.Position, angleNative,
            DemandType.ArbitraryFeedForward,
                Math.abs(FalconUtil.radiansToPosition(angleDelta, kSteerGearRatio)) > kAllowableSteeringError ?
                Math.copySign(kSteerFF.ks, angleDelta)/kVoltageSaturation
                :
                0
        );
        */
        double steerPID = steerController.calculate(getAbsoluteHeading().getRadians(), desiredState.angle.getRadians());
        double steerFF = kSteerFF.calculate(steerController.getSetpoint().velocity);
        steerMotor.set(ControlMode.PercentOutput, (steerFF + steerPID)/ kVoltageSaturation);

        // convert our target meters per second to falcon velocity units
        double velocityNative = FalconUtil.metersToVelocity(
            desiredState.speedMetersPerSecond,
            kDriveGearRatio,
            kWheelCircumference
        );
        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        driveMotor.set(
            ControlMode.Velocity, velocityNative, // Native falcon counts per 100ms
            DemandType.ArbitraryFeedForward, driveFF.calculate(desiredState.speedMetersPerSecond)/kVoltageSaturation // feedforward voltage to percent output
        );
    }
    public void setOpenLoopState(SwerveModuleState desiredState, boolean steerInPlace){
        Rotation2d currentRotation = getIntegratedHeading();
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);
        
        // our desired angle in [-pi, pi]
        double targetConstrainedAngle = desiredState.angle.getRadians();
        // our total current angle. This is not constrained to [-pi, pi]
        double currentTotalAngle = currentRotation.getRadians();
        // our current angle in [-pi, pi]
        double currentConstrainedAngle = MathUtil.angleModulus(currentTotalAngle);
        double angleDelta = MathUtil.angleModulus(targetConstrainedAngle - currentConstrainedAngle);
        // convert our constrained target to the closest "total" angle near our current total
        double targetTotalAngle = currentTotalAngle + angleDelta;

        // if the module is not driving, maintain last angle setpoint
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.04){
            targetTotalAngle = lastTargetTotalAngle;
            this.desiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        }
        else{
            lastTargetTotalAngle = targetTotalAngle;
            this.desiredState = desiredState;
        }

        // convert our target radians to falcon position units
        double angleNative = FalconUtil.radiansToPosition(targetTotalAngle, kSteerGearRatio);
        // perform onboard PID to steer the module to the target angle
        /*steerMotor.set(
            ControlMode.Position, angleNative,
            DemandType.ArbitraryFeedForward,
                Math.abs(FalconUtil.radiansToPosition(angleDelta, kSteerGearRatio)) > kAllowableSteeringError ?
                Math.copySign(kSteerFF.ks, angleDelta)/kVoltageSaturation
                :
                0
        );
        */
        double steerPID = steerController.calculate(getAbsoluteHeading().getRadians(), this.desiredState.angle.getRadians());
        double steerFF = kSteerFF.calculate(steerController.getSetpoint().velocity);
        steerMotor.set(ControlMode.PercentOutput, (steerFF + steerPID)/ kVoltageSaturation);

        // convert our target meters per second to falcon velocity units
        double velocityNative = FalconUtil.metersToVelocity(
            desiredState.speedMetersPerSecond,
            kDriveGearRatio,
            kWheelCircumference
        );
        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        driveMotor.set(
            ControlMode.Velocity, velocityNative, // Native falcon counts per 100ms
            DemandType.ArbitraryFeedForward, driveFF.calculate(desiredState.speedMetersPerSecond)/kVoltageSaturation // feedforward voltage to percent output
        );
    }
    public void setSteerVelocity(double steerVelocity){
        double steerFF = kSteerFF.calculate(steerVelocity);
        steerMotor.set(ControlMode.PercentOutput, (steerFF)/ kVoltageSaturation);
    }

    public void setDriveBrake(boolean is){
        driveMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }
    public void setSteerBrake(boolean is){
        steerMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Module heading reported by steering motor integrated encoder.
     * <br></br>
     * NOT constrained to [-pi, pi]
     */
    public Rotation2d getIntegratedHeading(){
        return Rotation2d.fromDegrees(FalconUtil.positionToDegrees(steerMotor.getSelectedSensorPosition(), kSteerGearRatio));
    }
    /**
     * Module heading reported by steering cancoder
     */
    public Rotation2d getAbsoluteHeading(){
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    /**
     * @return State describing integrated module rotation and velocity in meters per second
     */
    public SwerveModuleState getIntegratedState(){
        double velocity = FalconUtil.velocityToMeters(
            driveMotor.getSelectedSensorVelocity(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getIntegratedHeading();
        return new SwerveModuleState(velocity, angle);
    }
    public double getSteerVelocity(){
        return FalconUtil.velocityToRadians(steerMotor.getSelectedSensorVelocity(), kSteerGearRatio);
    }
    /**
     * @return State describing absolute module rotation and velocity in meters per second
     */

    public SwerveModuleState getAbsoluteState(){
        double velocity = FalconUtil.velocityToMeters(
            driveMotor.getSelectedSensorVelocity(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getAbsoluteHeading();
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * @return Constants about this module, like motor IDs, cancoder angle offset, and translation from center
     */
    public SwerveConstants.Module getModuleConstants(){
        return moduleConstants;
    }

    public void log(){
        SwerveModuleState state = getAbsoluteState();
        int num = moduleConstants.moduleNum;
        //SmartDashboard.putNumber("Module "+num+" Cancoder Degrees", getCancoderHeading().getDegrees());
        SmartDashboard.putNumber("Module "+num+" Steer Degrees", state.angle.plus(new Rotation2d()).getDegrees());
        SmartDashboard.putNumber("Module "+num+" Steer Native", steerMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Module "+num+" Steer Target Degrees", Units.radiansToDegrees(steerController.getSetpoint().position));
        //SmartDashboard.putNumber("Module "+num+" Steer Target Native", steerMotor.getClosedLoopTarget());
        //SmartDashboard.putNumber("Module "+num+" Steer Velocity", steerMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Module "+num+" Steer Target Velocity", Units.radiansToDegrees(steerController.getSetpoint().velocity));
        SmartDashboard.putNumber("Module "+num+" Steer Velocity", Units.radiansToDegrees(getSteerVelocity()));
        SmartDashboard.putNumber("Module "+num+" Drive Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
        SmartDashboard.putNumber("Module "+num+" Drive Velocity Target Feet", Units.metersToFeet(desiredState.speedMetersPerSecond));
    }


    // Simulation
    private final TalonFXSimCollection driveMotorSim;
    private final FlywheelSim driveMotorSimModel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            driveFF.kv * kWheelCircumference / (2*Math.PI),
            driveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        kDriveGearRatio
    );
    private final TalonFXSimCollection steerMotorSim;
    private final FlywheelSim steerMotorSimModel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(steerFF.kv, steerFF.ka),
        DCMotor.getFalcon500(1),
        kSteerGearRatio
    );
    private final CANCoderSimCollection steerEncoderSim;

    public void simulationPeriodic(){
        driveMotorSimModel.setInputVoltage(driveMotor.getMotorOutputVoltage());
        steerMotorSimModel.setInputVoltage(steerMotor.getMotorOutputVoltage());
        driveMotorSimModel.update(0.02);
        steerMotorSimModel.update(0.02);

        //SmartDashboard.putNumber("Drive Sim Model Amps", driveMotorSimModel.getCurrentDrawAmps());
        //SmartDashboard.putNumber("Drive Sim Model Velocity Feet", Units.metersToFeet(driveMotorSimModel.getAngularVelocityRPM() * kWheelCircumference / 60));
        double driveMotorVelocityNative = FalconUtil.rotationsToVelocity(driveMotorSimModel.getAngularVelocityRPM()/60, kDriveGearRatio);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setIntegratedSensorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addIntegratedSensorPosition((int)(driveMotorPositionDeltaNative));

        //SmartDashboard.putNumber("Steer Sim Model Velocity", steerMotorSimModel.getAngularVelocityRPM());
        double steerMotorVelocityNative = FalconUtil.rotationsToVelocity(steerMotorSimModel.getAngularVelocityRPM()/60, kSteerGearRatio);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setIntegratedSensorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addIntegratedSensorPosition((int)(steerMotorPositionDeltaNative));
        
        steerEncoderSim.setVelocity((int)(FalconUtil.rotationsToVelocity(steerMotorSimModel.getAngularVelocityRPM()/60, 1)*2));
        steerEncoderSim.setRawPosition((int)(getIntegratedHeading().getDegrees()/360.0*4096));

        driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
    }
}
