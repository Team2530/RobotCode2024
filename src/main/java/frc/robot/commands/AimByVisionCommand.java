package frc.robot.commands;

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
            
            int shoulder = 0;
            int wrist = 0;
            arm.setArmAngles(shoulder, wrist);
        } else if (Timer.getFPGATimestamp() - stallTimer > 1.5) {
            arm.setArmPreset(Presets.STOW);
        }
    }
    
}
