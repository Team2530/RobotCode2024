package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.*;

import java.util.Dictionary;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTag;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.KnownAprilTagDetail;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm.Presets;    


public class AimByOdometry extends Command {

    private SwerveSubsystem swerveSubsystem;
    private Arm arm;
    private AprilTagType target;

    private KnownAprilTagDetail aprilTagDetail;
    private AprilTag aprilTag;
    private Pose3d targetPose;
    private double relativeTargetHeight;
    private double relativeTargetDistance;
    private double targetAngle;
    private Pose2d robotPose;

    public AimByOdometry(SwerveSubsystem swerveSubsystem, Arm arm, AprilTagType target) {
        this.swerveSubsystem = swerveSubsystem;
        this.arm = arm;
        this.target = target;

        for (AprilTag x : Constants.AllAprilTags.values()) {
            if (x.GetTagType() == this.target && DriverStation.getAlliance().get() == x.GetAlliance()) {
                this.aprilTag = x;
                break;            
            }
        }
        this.targetPose = aprilTag.getTagPose3dinField();
    }

    @Override 
    public void execute() {
        relativeTargetDistance =  Math.sqrt(Math.pow(targetPose.getX() - robotPose.getX(),2)
                + Math.pow(targetPose.getY() - robotPose.getY(), 2)
        );
        if (relativeTargetDistance > 60) {
            robotPose = swerveSubsystem.getPose();

            relativeTargetHeight = targetPose.getZ() - MAST_HEIGHT;  
            targetAngle = targetPose.getRotation().getX() != 0.0
                ? targetPose.getRotation().getX()
                : targetPose.getRotation().getY(); 

            double shoulder;
            for (shoulder = 10; shoulder <= 90; shoulder += 5) { 
                if (calculateWrist(shoulder)+shoulder-targetAngle > calculateWrist(-5)+shoulder-targetAngle) {
                    break;
                }                   
            } 
            double wrist = calculateWrist(shoulder);
            arm.setArmAngles(shoulder, wrist);

            double relativeTargetAngle = Math.atan2(targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
            ChassisSpeeds prevSpeeds = swerveSubsystem.getChassisSpeeds();
            prevSpeeds.omegaRadiansPerSecond = DriveConstants.ORIENTED_PID.calculate(0, relativeTargetAngle);

            SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(prevSpeeds);
            swerveSubsystem.setModules(calculatedModuleStates);
        } else {
            arm.setArmPreset(Presets.STOW);
        }
    }

    private double calculateWrist(double shoulder) { 
        return Math.atan(
                (relativeTargetHeight - STAGE_ONE_LENGTH * Math.sin(shoulder))
                / (relativeTargetDistance - STAGE_ONE_LENGTH * Math.cos(shoulder))
            ) - shoulder; 
    }

   
}
