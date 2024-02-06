package frc.robot;

import java.time.LocalDateTime;
import edu.wpi.first.math.geometry.Pose3d;

public class KnownAprilTag {
    private double _tagId;
    private LocalDateTime _tagCaptureTime;
    private Pose3d _tagPose3d;
    public KnownAprilTag(double tagId, LocalDateTime tagCaptureTime, Pose3d tagPose3d){
        _tagId = tagId;
        _tagCaptureTime = tagCaptureTime;
        _tagPose3d = tagPose3d;
    }

    public void SetTagCaptureTime(LocalDateTime tagCaptureTime){
        _tagCaptureTime = tagCaptureTime;
    }
    
    public LocalDateTime GetTagCaptureTime(){
        return _tagCaptureTime;
    }

    public void SetTagPose3d(Pose3d tagPose3d){
        _tagPose3d = tagPose3d;
    }

    public Pose3d GetTagPose3d(){
        return _tagPose3d;
    }

    public double GetTagId(){
        return _tagId;
    }
}
