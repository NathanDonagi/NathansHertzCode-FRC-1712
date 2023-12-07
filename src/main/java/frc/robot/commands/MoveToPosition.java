package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosition extends CommandBase{
    SwerveSubsystem swerveSubsystem;
    Pose2d targetPos;
    Double duration;
    PIDController pidController;

    public MoveToPosition(SwerveSubsystem swerveSubsystem, Pose2d targetPos){
        // this.pidController = new  turningPidController = new PIDController(1, 0, 0);
        this.swerveSubsystem = swerveSubsystem;
        this.targetPos = targetPos;
    }

    public void initialize(){
    }

    public void execute(){
        swerveSubsystem.setSpeed(0.3);
        Transform2d posDiff = targetPos.minus(swerveSubsystem.getPose());
        double magnitude = Math.sqrt(posDiff.getX()*posDiff.getX()+posDiff.getY()*posDiff.getY());
// targetPos.getRotation().minus(swerveSubsystem.getRobotRotation2d()).getRadians()

        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0.5, swerveSubsystem.getRobotRotation2d());
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(posDiff.getX()/magnitude, posDiff.getY()/magnitude, 1, swerveSubsystem.getRobotRotation2d());

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }

    

    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }
    
    public boolean isFinished(){
        Transform2d posDiff = targetPos.minus(swerveSubsystem.getPose());
        double dx = posDiff.getX();
        double dy = posDiff.getY();
        return (dx*dx + dy*dy < 0.02);
    }
}
