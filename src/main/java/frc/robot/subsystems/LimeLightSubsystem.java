package frc.robot.subsystems;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KnownAprilTag;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimeLightSubsystem extends SubsystemBase {
    private Boolean hasAprilTag = false;
    private Boolean hasRetroTape = false;
    Dictionary<String, KnownAprilTag> lastKnownAprilTags = new Hashtable<String, KnownAprilTag>();

    public LimeLightSubsystem() {
    }

    @Override
    public void periodic() {
        try{
            KnownAprilTag aprilTag;
            LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults((LimelightConstants.limeLightName));
            
            //If we see april tag(s), we update the dictionary if needed, with position, capture time, and tag ID
            if(results.targetingResults.targets_Fiducials.length > 0){
                hasAprilTag = true;
                for (LimelightTarget_Fiducial fd : results.targetingResults.targets_Fiducials){
                    aprilTag = lastKnownAprilTags.get(fd.fiducialID);
                    //If the april tag doesn't already exist in the dictionary, add it
                    if(aprilTag == null){
                        aprilTag = new KnownAprilTag(fd.fiducialID, LocalDateTime.now(), fd.getTargetPose_RobotSpace());
                    }
                    else{ //If it DOES exist, make sure it's up to date 
                        aprilTag.SetTagCaptureTime(LocalDateTime.now());
                        aprilTag.SetTagPose3d(fd.getTargetPose_RobotSpace());
                    }
                    lastKnownAprilTags.put(String.valueOf(fd.fiducialID), aprilTag);
                }
                Enumeration<String> e = lastKnownAprilTags.keys();
                //Loops through dictionary of april tags, and if it's been over a second from capture time, removes the tag
                while(e.hasMoreElements()) {
                    String key = e.nextElement();
                    aprilTag = lastKnownAprilTags.get(key);
                    Duration duration = Duration.between(LocalDateTime.now(), aprilTag.GetTagCaptureTime());
                    if(duration.getSeconds() * 1000 >= 40){
                        lastKnownAprilTags.remove(key);
                    }
                }
                if(LimelightConstants.LOG_APRIL_TAGS_INTO_SMARTDASH_BOARD){
                    e = lastKnownAprilTags.keys();
                    //Loops through values of dictionary, for each april tag, prints ID as well as positioning/rotation
                    while(e.hasMoreElements()) {
                        String key = e.nextElement();
                        aprilTag = lastKnownAprilTags.get(key);
                        if(aprilTag != null){
                            Pose3d pose3d = aprilTag.GetTagPose3d();
                            SmartDashboard.putNumber("April Tag "+ aprilTag.GetTagId() + " X", pose3d.getZ());
                            SmartDashboard.putNumber("April Tag "+ aprilTag.GetTagId() + " Y", pose3d.getX());
                            SmartDashboard.putNumber("April Tag "+ aprilTag.GetTagId() +" Rotation", pose3d.getRotation().getY());
                        }
                    }
                }
            }
            else{
                hasAprilTag = false;
            }
        }
        catch(Exception e){
            SmartDashboard.putString("LimeLight Read error", e.getMessage());
        }
    }

    public Boolean isAprilTagFound() {
        return hasAprilTag;
    }

    public Boolean isRetorTapeFound() {
        return hasRetroTape;
    }

    public KnownAprilTag getKnownAprilTag(boolean isItForRightSideAprilTag)
    {
        String[] tagIdsForThisAction = {};
        Optional<Alliance> alliance = DriverStation.getAlliance();
        KnownAprilTag aprilTag = null;
        if(alliance.isPresent()){
            if(alliance.get() == Alliance.Blue){
                if(isItForRightSideAprilTag){
                    tagIdsForThisAction = AprilTags.BLUE_ALLIANCE_RIGHT_APRILTAGS;
                }
                else{
                    tagIdsForThisAction = AprilTags.BLUE_ALLIANCE_LEFT_OR_SINGLE_APRILTAGS;
                }
            }
            else if(alliance.get() == Alliance.Red){
                if(isItForRightSideAprilTag){
                    tagIdsForThisAction = AprilTags.RED_ALLIANCE_RIGHT_APRILTAGS;
                }
                else{
                    tagIdsForThisAction = AprilTags.RED_ALLIANCE_LEFT_OR_SINLGE_APRILTAGS;
                }
            }
            for (int i = 0; i < tagIdsForThisAction.length; i++) {
                aprilTag = lastKnownAprilTags.get(tagIdsForThisAction[i]);
                SmartDashboard.putString("I am inside 1 : ", tagIdsForThisAction[i]);
                if(aprilTag != null){
                    SmartDashboard.putString("Found aprilTag: ", "Id : " + aprilTag.GetTagId());
                    break;
                }
            }
        }
        return aprilTag;
    }
}
