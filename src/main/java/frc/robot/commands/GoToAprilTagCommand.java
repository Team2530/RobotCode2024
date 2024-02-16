package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionContsants;
import frc.robot.KnownAprilTagDetail;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;
import frc.robot.Constants.CommonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GoToAprilTagCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    public final LimeLightSubsystem limeLightSubsystem;
    public final AprilTagPosition tagPosition;
    public final AprilTagType searchTagType;

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(VisionContsants.X_kP, VisionContsants.X_kI, VisionContsants.X_kD, LimelightConstants.pidXConstriants);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(VisionContsants.Y_kP, VisionContsants.Y_kI, VisionContsants.Y_kD, LimelightConstants.pidYConstraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(VisionContsants.THETA_kP, VisionContsants.THETA_kI, VisionContsants.THETA_kD, LimelightConstants.pidOmegaConstraints);
    
    StringLogEntry log;
    Boolean isSearchTagFound = false;

    public GoToAprilTagCommand(
        SwerveSubsystem swerveSubsystem,
        LimeLightSubsystem limeLightSubsystem,
        AprilTagPosition tagPosition,
        AprilTagType searchTagType) 
    {
        this.swerveSubsystem = swerveSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;
        this.tagPosition = tagPosition;
        this.searchTagType = searchTagType;
        addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    isSearchTagFound = false;
    
    //pidControllerX.reset(swerveSubsystem.getPose().getY());
    //pidControllerY.reset(swerveSubsystem.getPose().getX());
    //pidControllerOmega.reset(swerveSubsystem.getRotation2d().getRadians());
    

    pidControllerX.setGoal(Units.inchesToMeters(50)); // Move forward/backwork to keep 36 inches from the target
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerY.setGoal(0); // Move side to side to keep target centered
    pidControllerY.setTolerance(Units.inchesToMeters(2.5));

    pidControllerOmega.setGoal(Units.degreesToRadians(0)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));

    if(CommonConstants.LOG_INTO_FILE_ENABLED){
      /// Starts recording to data log
      try {
        String directory = Paths.get("").toAbsolutePath().toString();
        Files.createDirectories(Path.of(directory));
        DataLogManager.start(directory);
      } catch (IOException e) {
        SmartDashboard.putString("AprilTag Log error", e.getMessage());
      }
      // Record both DS control and joystick data
      DriverStation.startDataLog(DataLogManager.getLog());
      log = new StringLogEntry(DataLogManager.getLog(), "GoToAprilTagCommand");
    }

  }

  @Override
  public void execute() {
    try{
      ChassisSpeeds speeds;
      if(limeLightSubsystem.isAprilTagFound()){
        KnownAprilTagDetail aprilTag = null;
        if(searchTagType == null){
          aprilTag = limeLightSubsystem.getKnownAprilTagDetail(tagPosition);
        }
        else{
          aprilTag = limeLightSubsystem.getKnownAprilTagDetailByType(searchTagType);
        }
        if(aprilTag != null){
          if(searchTagType !=  null){
            isSearchTagFound = true;
          }
          Pose3d pose3d = aprilTag.GetTargetPoseRobotSpace();
          if(CommonConstants.LOG_INTO_FILE_ENABLED){
            SmartDashboard.putNumber("Current April Tag "+ aprilTag.GetAprilTag().GetTagId() + " X", pose3d.getZ());
            SmartDashboard.putNumber("Current April Tag "+ aprilTag.GetAprilTag().GetTagId() + " Y", pose3d.getX());
            SmartDashboard.putNumber("CurrentApril Tag "+ aprilTag.GetAprilTag().GetTagId() +" Rotation", pose3d.getRotation().getY());
          }
          var xspeed = pidControllerX.calculate(pose3d.getZ());          
          if (pidControllerX.atSetpoint()) {
            xspeed = 0;
          }
    
            // Handle alignment side-to-side
          var ySpeed = pidControllerY.calculate(pose3d.getX());
          if (pidControllerY.atSetpoint()) {
            ySpeed = 0;
          }

          // Handle rotation using target Yaw/Z rotation
          var omegaSpeed = pidControllerOmega.calculate(pose3d.getRotation().getY());
          if (pidControllerOmega.atSetpoint()) {
            omegaSpeed = 0;
          }
    
          if(CommonConstants.LOG_INTO_FILE_ENABLED){
            String logMessage = "target X: " + pose3d.getX() + ": ";
            logMessage += "target X(inches): " + Units.metersToInches(pose3d.getZ()) + ": ";
            logMessage += "X speed : " + xspeed + ": ";
            logMessage += "X SetPoint : " + pidControllerX.getSetpoint() + ": ";
            logMessage += "X is at SetPoint : " + pidControllerX.atSetpoint() + ": ";
            logMessage += "target Y: " + pose3d.getX() + ": ";
            logMessage += "Y speed : " + ySpeed + ": ";
            logMessage += "Y SetPoint : " + pidControllerY.getSetpoint() + ": ";
            logMessage += "Y is at SetPoint : " + pidControllerY.atSetpoint() + ": ";
            logMessage += "target rotation: " + pose3d.getRotation().getY() + ": ";
            logMessage += "rotation speed : " + omegaSpeed + ": ";
            logMessage += "rotation SetPoint : " + pidControllerOmega.getSetpoint() + ": ";
            logMessage += "rotation is at SetPoint : " + pidControllerOmega.atSetpoint() + ": ";
            log.append(logMessage);
          }          
          //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xspeed, -ySpeed, -zSpeed, swerveSubsystem.getRotation2d());
          speeds = new ChassisSpeeds(-ySpeed, xspeed, omegaSpeed);

          SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
          swerveSubsystem.setModules(calculatedModuleStates);
        } 
        else{
          if(searchTagType != null && !isSearchTagFound){
            speeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(LimelightConstants.APRILTAG_SEARCH_ROTATION));
            SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
            swerveSubsystem.setModules(calculatedModuleStates);
          }
          else{
            //swerveSubsystem.stopDrive();
          }
        }
      }
      else{
        if(searchTagType != null && !isSearchTagFound){
          speeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(LimelightConstants.APRILTAG_SEARCH_ROTATION));
          SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
          swerveSubsystem.setModules(calculatedModuleStates);
        }
        else{
          //swerveSubsystem.stopDrive();
        }
      }
    }
    catch(Exception e){
      SmartDashboard.putString("AprilTag chase command error", e.getMessage());
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

}
