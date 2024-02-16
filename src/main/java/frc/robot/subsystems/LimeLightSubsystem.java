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
import frc.robot.AprilTag;
import frc.robot.Constants;
import frc.robot.KnownAprilTagDetail;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimeLightSubsystem extends SubsystemBase {
    private Boolean hasAprilTag = false;
    private Boolean hasRetroTape = false;
    Dictionary<String, KnownAprilTagDetail> lastKnownAprilTagDetails = new Hashtable<String, KnownAprilTagDetail>();

    public LimeLightSubsystem() {
    }

    @Override
    public void periodic() {
        try{
            KnownAprilTagDetail aprilTagDetail;
            AprilTag tag;
            LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults((LimelightConstants.limeLightName));
            //If we see april tag(s), we update the dictionary if needed, with position, capture time, and tag ID
            if(results.targetingResults.targets_Fiducials.length > 0){
                hasAprilTag = true;
                for (LimelightTarget_Fiducial fd : results.targetingResults.targets_Fiducials){
                    aprilTagDetail = lastKnownAprilTagDetails.get(fd.fiducialID);
                    tag = Constants.AllAprilTags.get(Double.toString(fd.fiducialID));
                    if(tag != null)
                    {
                        SmartDashboard.putString("Number of tags 1: ", tag.GetTagId());
                    }
                    //If the april tag doesn't already exist in the dictionary, add it
                    if(aprilTagDetail == null && tag != null){
                         aprilTagDetail = new KnownAprilTagDetail(tag, LocalDateTime.now(), fd.getTargetPose_RobotSpace());
                    }
                    else if(aprilTagDetail != null){ //If it DOES exist, make sure it's up to date 
                        aprilTagDetail.SetTagCaptureTime(LocalDateTime.now());
                        aprilTagDetail.SetTargetPoseRobotSpace(fd.getTargetPose_RobotSpace());
                    }
                    if(aprilTagDetail != null){
                        lastKnownAprilTagDetails.put(String.valueOf(fd.fiducialID), aprilTagDetail);
                    }
                }
            }
            else{
                hasAprilTag = false;
            }
            Enumeration<String> e = lastKnownAprilTagDetails.keys();
                //Loops through dictionary of april tags, and if it's been over a second from capture time, removes the tag
                while(e.hasMoreElements()) {
                    String key = e.nextElement();
                    aprilTagDetail = lastKnownAprilTagDetails.get(key);
                    Duration duration = Duration.between(aprilTagDetail.GetTagCaptureTime(), LocalDateTime.now());
                    if(duration.toMillis() >=  LimelightConstants.CLEAR_APRILTAG_INTERVAL){
                        lastKnownAprilTagDetails.remove(key);
                    }
                }
                if(LimelightConstants.LOG_APRIL_TAGS_INTO_SMARTDASH_BOARD){
                    e = lastKnownAprilTagDetails.keys();
                    //Loops through values of dictionary, for each april tag, prints ID as well as positioning/rotation
                    while(e.hasMoreElements()) {
                        String key = e.nextElement();
                        aprilTagDetail = lastKnownAprilTagDetails.get(key);
                        if(aprilTagDetail != null){
                            Pose3d pose3d = aprilTagDetail.GetTargetPoseRobotSpace();
                            SmartDashboard.putNumber("April Tag "+ aprilTagDetail.GetAprilTag().GetTagId() + " X", pose3d.getZ());
                            SmartDashboard.putNumber("April Tag "+ aprilTagDetail.GetAprilTag().GetTagId() + " Y", pose3d.getX());
                            SmartDashboard.putNumber("April Tag "+ aprilTagDetail.GetAprilTag().GetTagId() +" Rotation", pose3d.getRotation().getY());
                        }
                    }
                    SmartDashboard.putNumber("April Tags Count", lastKnownAprilTagDetails.size());
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

    public KnownAprilTagDetail getKnownAprilTagDetail(AprilTagPosition tagPosition)
    {
        // if there is an alliance it gets the alliance (blue or red)
        Optional<Alliance> alliance = DriverStation.getAlliance();
        KnownAprilTagDetail returnValue = null;
        // loops through the hashtable and finds the correct apriltag and returns the details
        if(alliance.isPresent()){
            Enumeration<String> e = lastKnownAprilTagDetails.keys();
            while(e.hasMoreElements()) {
                String key = e.nextElement();
                KnownAprilTagDetail aprilTagDetail = lastKnownAprilTagDetails.get(key);
                if(aprilTagDetail != null && aprilTagDetail.GetAprilTag().GetAlliance() == alliance.get() && aprilTagDetail.GetAprilTag().GetTagPosition() == tagPosition){
                    returnValue = aprilTagDetail;
                    break;
                }
            }
        }
        return returnValue;
    }

    public KnownAprilTagDetail getKnownAprilTagDetailByType(AprilTagType tagType)
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        KnownAprilTagDetail returnValue = null;
        if(alliance.isPresent()){
            Enumeration<String> e = lastKnownAprilTagDetails.keys();
            while(e.hasMoreElements()) {
                String key = e.nextElement();
                KnownAprilTagDetail aprilTagDetail = lastKnownAprilTagDetails.get(key);
                if(aprilTagDetail != null && aprilTagDetail.GetAprilTag().GetAlliance() == alliance.get() && aprilTagDetail.GetAprilTag().GetTagType() == tagType){
                    returnValue = aprilTagDetail;
                    break;
                }
            }
        }
        return returnValue;
    }

}