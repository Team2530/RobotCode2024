package frc.robot;

import java.time.LocalDateTime;
import edu.wpi.first.math.geometry.Pose3d;

public class KnownAprilTag {
    private double tagId;
    private LocalDateTime tagCaptureTime;
    private Pose3d tagPose3d;
    public KnownAprilTag(double tagId, LocalDateTime tagCaptureTime, Pose3d tagPose3d){
        this.tagId = tagId;
        this.tagCaptureTime = tagCaptureTime;
        this.tagPose3d = tagPose3d;
    }

    public void SetTagCaptureTime(LocalDateTime tagCaptureTime){
        this.tagCaptureTime = tagCaptureTime;
    }
    
    public LocalDateTime GetTagCaptureTime(){
        return tagCaptureTime;
    }

    public void SetTagPose3d(Pose3d tagPose3d){
        this.tagPose3d = tagPose3d;
    }

    public Pose3d GetTagPose3d(){
        return tagPose3d;
    }

    public double GetTagId(){
        return tagId;
    }
}
