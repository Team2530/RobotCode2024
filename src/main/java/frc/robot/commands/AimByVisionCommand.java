package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTag;
import frc.robot.Constants;
import frc.robot.KnownAprilTagDetail;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm.Presets;    


public class AimByVisionCommand extends Command {

    private SwerveSubsystem swerveSubsystem;
    private Arm arm;
    private LimeLightSubsystem limeLightSubsystem;
    
    private double stallTimer;

    private KnownAprilTagDetail aprilTagDetail;
    private AprilTag aprilTag;
    private Pose3d targetPose;
    private double relativeTargetHeight;
    private double relativeTargetDistance;
    private double targetAngle;
    private Pose2d robotPose;

    public AimByVisionCommand(SwerveSubsystem swerveSubsystem, Arm arm, LimeLightSubsystem limeLightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.arm = arm;
        this.limeLightSubsystem = limeLightSubsystem;

        stallTimer = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute() {
        if (limeLightSubsystem.isAprilTagFound()) {
            aprilTagDetail = limeLightSubsystem.getKnownAprilTagDetail(AprilTagPosition.CENTER);
            aprilTag = aprilTagDetail.GetAprilTag();
            robotPose = swerveSubsystem.getPose();
            switch (aprilTag.GetAlliance()) {
                case Red:
                    targetPose = Constants.AprilTagTargetsRed.get(aprilTag.GetTagType());
                    break;
                case Blue:
                    targetPose = Constants.AprilTagTargetsBlue.get(aprilTag.GetTagType());
                    break;
            }
            relativeTargetDistance =  Math.sqrt(Math.pow(Math.abs(targetPose.getX() - robotPose.getX()),2)
                + Math.pow(Math.abs(targetPose.getY() - robotPose.getY()), 2)
            );
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
        } else if (Timer.getFPGATimestamp() - stallTimer > 1.5) {
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
