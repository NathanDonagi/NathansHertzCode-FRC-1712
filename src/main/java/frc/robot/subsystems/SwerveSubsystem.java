package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystickCMD;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.frontLeftDriveMotorPort,
        DriveConstants.frontLeftTurningMotorPort,
        false, 
        false,
        DriveConstants.frontLeftAbsoluteEncoder, 0.75, 0, 0);
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.frontRightDriveMotorPort,
        DriveConstants.frontRightTurningMotorPort,
        false,
        false,
        DriveConstants.frontRightAbsoluteEncoder, 0.75, 0, 0);
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.backLeftDriveMotorPort,
        DriveConstants.backLeftTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.75, 0, 0);
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.backRightDriveMotorPort,
        DriveConstants.backRightTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.75, 0, 0);

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
    private SwerveDriveOdometry swerveDriveOdometry;

    private double FLTarget, FRTarget, BLTarget, BRTarget;

    public boolean locked;
    public boolean teleop;

public SwerveSubsystem(){
        // gyro.
        swerveDriveOdometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics, 
            getRobotRotation2d(), 
            getModulePositions());
        locked = false;
        teleop = true;
        new Thread(() -> {
            try{
                Thread.sleep(1000);
            } catch(Exception e){}
            zeroHeading();
        }).start();
    }

    public SwerveModule getFL(){
        return frontLeft;
    }
    public SwerveModule getFR(){
        return frontRight;
    }
    public SwerveModule getBL(){
        return backLeft;
    }
    public SwerveModule getBR(){
        return backRight;
    }

    public void zeroHeading(){
        gyro.reset();
    }
    public double getHeading(){
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Pose2d getPose(){
        return swerveDriveOdometry.getPoseMeters();
    }

    public Rotation2d getRobotRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void periodic(){
        swerveDriveOdometry.update(getRobotRotation2d(), getModulePositions());
        SmartDashboard.putNumber("robot x", getPose().getX());
        SmartDashboard.putNumber("robot y", getPose().getY());

        // SmartDashboard.putNumber("Front Left Target Rotation", FLTarget);
        // SmartDashboard.putNumber("Front Right Target Rotation", FRTarget);
        // SmartDashboard.putNumber("Back Left Target Rotation", BLTarget);
        // SmartDashboard.putNumber("Back Right Target Rotation", BRTarget);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left Rotation", frontLeft.getTurnPosition());
        SmartDashboard.putNumber("Front Right Rotation", frontRight.getTurnPosition());
        SmartDashboard.putNumber("Back Left Rotation", backLeft.getTurnPosition());
        SmartDashboard.putNumber("Back Right Rotation", backRight.getTurnPosition());

        SmartDashboard.putNumber("Front Left Abs Encoder", frontLeft.getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Front Right Abs Encoder", frontRight.getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Back Left Abs Encoder", backLeft.getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Back Right Abs Encoder", backRight.getAbsoluteEncoderAngle());

        // SmartDashboard.putNumber("Front Left Turn Output", frontLeft.getTurnMotorOutput());
        // SmartDashboard.putNumber("Front Right Turn Output", frontRight.getTurnMotorOutput());
        // SmartDashboard.putNumber("Back Left Turn Output", backLeft.getTurnMotorOutput());
        // SmartDashboard.putNumber("Back Right Turn Output", backRight.getTurnMotorOutput());
    }

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(), backRight.getModulePosition()};
    }

    public void resetPose(){
        swerveDriveOdometry.resetPosition(getRobotRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    }

    
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public void setSpeed(double speed){
        frontLeft.setPercentSpeed(speed);
        frontRight.setPercentSpeed(speed);
        backLeft.setPercentSpeed(speed);
        backRight.setPercentSpeed(speed);
    }
    public void setModuleStates(SwerveModuleState[] states){
        FLTarget = states[0].angle.getRadians();
        FRTarget = states[1].angle.getRadians();
        BLTarget = states[2].angle.getRadians();
        BRTarget = states[3].angle.getRadians();

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
}
