package frc.robot;

import java.time.LocalDateTime;
import edu.wpi.first.math.geometry.Pose3d;

public class KnownAprilTag {
    private final double tagId;
    private LocalDateTime tagCaptureTime;
    private Pose3d tagPose3d;
    public KnownAprilTag(double tagId, LocalDateTime tagCaptureTime, Pose3d tagPose3d){
        this.tagId = tagId;
        this.tagCaptureTime = tagCaptureTime;
        this.tagPose3d = tagPose3d;
    }

    public void setTagCaptureTime(LocalDateTime tagCaptureTime){
        this.tagCaptureTime = tagCaptureTime;
    }
    
    public LocalDateTime getTagCaptureTime(){
        return tagCaptureTime;
    }

    public void setTagPose3d(Pose3d tagPose3d){
        this.tagPose3d = tagPose3d;
    }

    public Pose3d getTagPose3d(){
        return tagPose3d;
    }

    public double getTagId(){
        return tagId;
    }
}
