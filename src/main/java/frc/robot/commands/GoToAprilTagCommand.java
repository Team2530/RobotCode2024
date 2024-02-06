package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionContsants;
import frc.robot.KnownAprilTag;
import frc.robot.Constants.CommonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import static edu.wpi.first.math.MathUtil.clamp;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GoToAprilTagCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    public final LimeLightSubsystem limeLightSubsystem;
    public final boolean isItForRightSideAprilTag;

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(VisionContsants.X_kP, VisionContsants.X_kI, VisionContsants.X_kD, new Constraints(DriveConstants.MAX_ROBOT_VELOCITY, DriveConstants.MAX_ROBOT_RAD_VELOCITY));
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(VisionContsants.Y_kP, VisionContsants.Y_kI, VisionContsants.Y_kD, new Constraints(DriveConstants.MAX_ROBOT_VELOCITY, DriveConstants.MAX_ROBOT_RAD_VELOCITY));
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(VisionContsants.THETA_kP, VisionContsants.THETA_kI, VisionContsants.THETA_kD, new Constraints(DriveConstants.MAX_ROBOT_VELOCITY, DriveConstants.MAX_ROBOT_RAD_VELOCITY));
    
    StringLogEntry log;

    public GoToAprilTagCommand(
        SwerveSubsystem swerveSubsystem,
        LimeLightSubsystem limeLightSubsystem,
        boolean isItForRightSideAprilTag) 
    {
        this.swerveSubsystem = swerveSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;
        this.isItForRightSideAprilTag = isItForRightSideAprilTag;
        addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    //pidControllerX.reset(swerveSubsystem.getPose().getY());
    //pidControllerY.reset(swerveSubsystem.getPose().getX());
    //pidControllerOmega.reset(swerveSubsystem.getRotation2d().getRadians());
    

    pidControllerX.setGoal(Units.inchesToMeters(36)); // Move forward/backwork to keep 36 inches from the target
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
      log = new StringLogEntry(DataLogManager.getLog(), "ChaseApriltagCommand");
    }

  }

  @Override
  public void execute() {
    try{
      ChassisSpeeds speeds;
      if(limeLightSubsystem.isAprilTagFound()){
        KnownAprilTag aprilTag = limeLightSubsystem.getKnownAprilTag(isItForRightSideAprilTag);
        if(aprilTag != null){
          Pose3d pose3d = aprilTag.GetTagPose3d();
          SmartDashboard.putNumber("Current April Tag "+ aprilTag.GetTagId() + " X", pose3d.getZ());
          SmartDashboard.putNumber("Current April Tag "+ aprilTag.GetTagId() + " Y", pose3d.getX());
          SmartDashboard.putNumber("CurrentApril Tag "+ aprilTag.GetTagId() +" Rotation", pose3d.getRotation().getY());
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
      }
    }
    catch(Exception e){
      SmartDashboard.putString("AprilTag chase command error", e.getMessage());
    }
  }

  public Translation2d DeadBand(Translation2d input, double deadzone) {
    double mag = input.getNorm();
    Translation2d norm = input.div(mag);

    if (mag < deadzone) {
        return new Translation2d(0.0, 0.0);
    } else {
        // TODO: Check is it sqrt2 or 1.0...
        Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
        return new Translation2d(
                clamp(result.getX(), -1.0, 1.0),
                clamp(result.getY(), -1.0, 1.0));
    }
}

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }
}

