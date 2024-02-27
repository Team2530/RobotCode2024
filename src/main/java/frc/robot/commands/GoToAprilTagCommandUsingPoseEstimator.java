package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionContsants;
import frc.robot.AprilTag;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;
import frc.robot.Constants.CommonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Supplier;

public class GoToAprilTagCommandUsingPoseEstimator extends Command {

    private static final double TRANSLATION_TOLERANCE = 2.5;
    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
    
    private final SwerveSubsystem swerveSubsystem;
    public final Supplier<Pose2d> robotPoseSupplier;
    public final AprilTagPosition tagPosition;
    public final AprilTagType tagType;
    public final AprilTag tag;

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(VisionContsants.X_kP, VisionContsants.X_kI, VisionContsants.X_kD, LimelightConstants.pidXConstriants);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(VisionContsants.Y_kP, VisionContsants.Y_kI, VisionContsants.Y_kD, LimelightConstants.pidYConstraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(VisionContsants.THETA_kP, VisionContsants.THETA_kI, VisionContsants.THETA_kD, LimelightConstants.pidOmegaConstraints);
    

    public GoToAprilTagCommandUsingPoseEstimator(
        SwerveSubsystem swerveSubsystem,
        Supplier<Pose2d> robotPoseSupplier,
        AprilTagPosition tagPosition,
        AprilTagType tagType) 
    {
        this.swerveSubsystem = swerveSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.tagPosition = tagPosition;
        this.tagType = tagType;
        this.tag = swerveSubsystem.getAprilTag(tagPosition, tagType);
        addRequirements(swerveSubsystem);
    }

  @Override
  public void initialize() {
    super.initialize();
    if(tag != null)
    {
      Pose2d goalPose = tag.getTagPose2dInFieldWithPreset();
      if(goalPose != null){
        if(CommonConstants.LOG_INTO_FILE_ENABLED){
        SmartDashboard.putNumber("PE - GoalPoseX", goalPose.getX());
        SmartDashboard.putNumber("PE - GoalPoseY", goalPose.getY());
        SmartDashboard.putNumber("PE - GoalPoseRotation", goalPose.getRotation().getRadians());
        
      }
        resetPIDControllers();
        pidControllerX.setGoal(goalPose.getX()); // Move forward/backwork to keep 36 inches from the target
        pidControllerX.setTolerance(TRANSLATION_TOLERANCE);

        pidControllerY.setGoal(goalPose.getY()); // Move side to side to keep target centered
        pidControllerY.setTolerance(TRANSLATION_TOLERANCE);

        pidControllerOmega.setGoal(goalPose.getRotation().getRadians()); // Rotate the keep perpendicular with the target
        pidControllerOmega.setTolerance(THETA_TOLERANCE);

        if(CommonConstants.LOG_INTO_FILE_ENABLED){
          /// Starts recording to data log
          try {
            String directory = Paths.get("").toAbsolutePath().toString();
            Files.createDirectories(Path.of(directory));
            DataLogManager.start(directory);
          } catch (IOException e) {
            SmartDashboard.putString("AprilTag Log error", e.getMessage());
          }
        }
      }
    }
  }

  @Override
  public void execute() {
    try{
        ChassisSpeeds speeds;
        Pose2d robotPose = robotPoseSupplier.get();
        if(robotPose != null){
         if(CommonConstants.LOG_INTO_FILE_ENABLED){
          if(tag != null){
            SmartDashboard.putString("Current tagID", tag.GetTagId());
          }
           
            SmartDashboard.putNumber("PE - Current pose X", robotPose.getX());
            SmartDashboard.putNumber("PE - Current pose Y", robotPose.getY());
            SmartDashboard.putNumber("PE - Current pose Rotation", robotPose.getRotation().getRadians());
          }
          var xSpeed = pidControllerX.calculate(robotPose.getX());          
          if (pidControllerX.atSetpoint()) {
            xSpeed = 0;
          }

          // Handle alignment side-to-side
          var ySpeed = 0;//pidControllerY.calculate(robotPose.getY());
          if (pidControllerY.atSetpoint()) {
            ySpeed = 0;
          }

          // Handle rotation using target Yaw/Z rotation
          var omegaSpeed = 0;//pidControllerOmega.calculate(robotPose.getRotation().getRadians());
          if (pidControllerOmega.atSetpoint()) {
            omegaSpeed = 0;
          }        
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, -xSpeed, -omegaSpeed, robotPose.getRotation());

          SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
          swerveSubsystem.setModules(calculatedModuleStates); 
        }
    }
    catch(Exception e){
      SmartDashboard.putString("AprilTag pose command error", e.getMessage());
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }

  public boolean atGoal() {
    return pidControllerX.atGoal() && pidControllerY.atGoal() && pidControllerOmega.atGoal();
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  private void resetPIDControllers() {
    var robotPose =  robotPoseSupplier.get();
    pidControllerOmega.reset(robotPose.getRotation().getRadians());
    pidControllerX.reset(robotPose.getX());
    pidControllerY.reset(robotPose.getY());
  }
}