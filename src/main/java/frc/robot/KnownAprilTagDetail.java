package frc.robot;

import java.time.LocalDateTime;
import edu.wpi.first.math.geometry.Pose3d;

public class KnownAprilTagDetail {
    private AprilTag _tag;
    private LocalDateTime _tagCaptureTime;
    private Pose3d _targetPoseRobotSpace;
    public KnownAprilTagDetail(AprilTag tag, LocalDateTime tagCaptureTime, Pose3d targetPoseRobotSpace){
        _tag = tag;
        _tagCaptureTime = tagCaptureTime;
        _targetPoseRobotSpace = targetPoseRobotSpace;
    }

    public void SetTagCaptureTime(LocalDateTime tagCaptureTime){
        _tagCaptureTime = tagCaptureTime;
    }
    
    public LocalDateTime GetTagCaptureTime(){
        return _tagCaptureTime;
    }

    public void SetTargetPoseRobotSpace(Pose3d targetPoseRobotSpace){
        _targetPoseRobotSpace = targetPoseRobotSpace;
    }

    public Pose3d GetTargetPoseRobotSpace(){
        return _targetPoseRobotSpace;
    }

    public AprilTag GetAprilTag() {
        return _tag;
    }
}
