package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;

public class AprilTag {
    private String _tagId;
    private AprilTagType _tagType;
    private AprilTagPosition _tagPosition;
    private Alliance _alliance;
    private Double _x;
    private Double _y;
    private Double _z;
    private double _rotation;
    // _preset is for the distance to stop from the target
    private double _presetX;
    private double _presetY;
    private double _presetRotation;

    public AprilTag(String tagId, AprilTagType tagType, AprilTagPosition tagPosition, Alliance alliance, double x, double y, double z, double rotation, double presetX, double presetY, double presetRotation){
        _tagId = tagId;
        _tagType = tagType;
        _tagPosition = tagPosition;
        _alliance = alliance;
        _x = x;
        _y = y;
        _z = z;
        _rotation = rotation;
        _presetX = presetX;
        _presetY = presetY;
        _presetRotation = presetRotation;
    }

    public AprilTagType GetTagType(){
        return _tagType;
    }

    public String GetTagId(){
        return _tagId;
    }

    public AprilTagPosition GetTagPosition(){
        return _tagPosition;
    }

    public Alliance GetAlliance(){
        return _alliance;
    }

    public Pose2d getTagPose2dInField(){
        return new Pose2d(new Translation2d(Units.inchesToMeters(_x), Units.inchesToMeters(_y)), new Rotation2d(Units.degreesToRadians(_rotation)));
    }

    public Pose2d getTagPose2dInFieldWithPreset(){
        return new Pose2d(new Translation2d(Units.inchesToMeters(_x - _presetX), Units.inchesToMeters(_y - _presetY)), new Rotation2d(Units.degreesToRadians(_rotation - _presetRotation)));
    }    

    public Pose2d getTagPose2dInFieldForGivenPreset(double presetX, double presetY, double presetRotation){
        return new Pose2d(new Translation2d(Units.inchesToMeters(_x - presetX), Units.inchesToMeters(_y - presetY)), new Rotation2d(Units.degreesToRadians(_rotation - presetRotation)));
    }    


    public double getPresetX(){
        return(_presetX);
    }

    public double getPresetY(){
        return(_presetY);
    }

    public double getPresetRotation(){
        return(_presetRotation);
    }
}