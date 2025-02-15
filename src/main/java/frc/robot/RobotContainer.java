package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.commands.FollowCircle;
import frc.robot.common.OCXboxController;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    
    private final Drivetrain drivetrain;

    private final OCXboxController driver = new OCXboxController(0);
    private boolean isFieldRelative = true;

    private final AutoOptions autoOptions;

    public RobotContainer(){
        drivetrain = new Drivetrain();

        configureDriverBinds();

        autoOptions = new AutoOptions(drivetrain);
        autoOptions.submit();
    }

    public void periodic(){
    }

    public Command getAutoCommand(){
        return autoOptions.getSelected();
    }

    public void disable(){
        drivetrain.drive(0, 0, 0, false);
    }

    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
    }

    private void configureDriverBinds(){
        // when no other command is using the drivetrain, we
        // pass the joysticks for forward, strafe, and angular control
        Command teleopDrive = new RunCommand(()->{
            drivetrain.drive(
                driver.getForward(),
                driver.getStrafe(),
                driver.getTurn(),
                isFieldRelative);
        }, drivetrain).beforeStarting(()->driver.resetLimiters());
        drivetrain.setDefaultCommand(teleopDrive);

        // push-to-change driving "speed"
        driver.rightBumper
            .whenPressed(()->driver.setDriveSpeed(OCXboxController.kSpeedMax))
            .whenReleased(()->driver.setDriveSpeed(OCXboxController.kSpeedDefault));

        // change from field-relative to robot-relative control
        driver.backButton.whenPressed(()->{
            isFieldRelative = !isFieldRelative;
        });

        // reset the robot heading to 0
        driver.startButton.whenPressed(()->{
            drivetrain.resetOdometry(
                new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        });

        // lock the modules in a "X" alignment
        driver.xButton.whileHeld(()->{
            SwerveModuleState[] states = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            };
            drivetrain.setModuleStates(states, true);
        }, drivetrain);

        // follow a 1.5 meter diameter circle in front of the robot
        // we use an instant command like this to construct a new command every button press
        driver.bButton.whenPressed(
            new InstantCommand(()->{
                new FollowCircle(drivetrain, 1.5, new Rotation2d(), AutoConstants.kSlowSpeedConfig).schedule();
            })
        );

        //driver.yButton.whenPressed(new InstantCommand(()->autoOptions.getSelected().schedule()));
        driver.yButton.whileHeld(()->{
            double steerVelocity = 4;
            drivetrain.setModuleSteerVelocity(new double[]{steerVelocity,steerVelocity,steerVelocity,steerVelocity});
        }, drivetrain);
    }

    public void log(){
        drivetrain.log();
        NetworkTableInstance.getDefault().flush();
    }
}
