package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTag;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FL_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FR_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BR_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BL_MOTOR_REVERSED);

    PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    int[] pdh_channels = {
            18, 19,
            0, 1,
            16, 17,
            2, 3
    };

    private final AHRS navX = new AHRS(SPI.Port.kMXP);
    private double navxSim;

    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private Field2d field = new Field2d();

    // TODO: Properly set starting pose
    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS,
            getRotation2d(),
            getModulePositions(), new Pose2d());

    public SwerveSubsystem() {
        zeroHeading();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
        
        try{
            var poseEntry = LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_wpiblue");
            var poseArray = poseEntry.getDoubleArray(new double[0]);
            if(poseArray.length > 0)
            {
                SmartDashboard.putNumber("I am here", poseArray.length);
                double timestamp = poseEntry.getLastChange() / 1e6 - poseArray[6] / 1e3;
                Pose2d pose = new Pose2d(
                new Translation2d(poseArray[0], poseArray[1]),
                new Rotation2d(Units.degreesToRadians(poseArray[5])));

                odometry.addVisionMeasurement(pose, timestamp);
            }
        }
        catch(Exception e){
            SmartDashboard.putString("AddvisionError", e.getMessage());
          }

        if(DriverStation.getAlliance().isPresent()) {
            switch (DriverStation.getAlliance().get()) {
                case Red:
                    field.setRobotPose(new Pose2d(new Translation2d(16.5 - getPose().getX(), 8.02 - getPose().getY()),
                        getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180)))); 
                    break;
            
                case Blue:
                    field.setRobotPose(getPose());
                    break;
            }
        } else {
            // If no alliance provided, just go with blue
            field.setRobotPose(getPose());
        }

        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());

        double swerveCurrent = 0;
        for (int chan : pdh_channels)
            swerveCurrent += pdh.getCurrent(chan);
        SmartDashboard.putNumber("SwerveSubsystem Amps", swerveCurrent);
        SmartDashboard.putNumber("PDH Amps", pdh.getTotalCurrent());

        SmartDashboard.putNumberArray("SwerveStates", new double[] {
                frontLeft.getModuleState().angle.getDegrees() + 90, -frontLeft.getModuleState().speedMetersPerSecond,
                frontRight.getModuleState().angle.getDegrees() + 90, -frontRight.getModuleState().speedMetersPerSecond,
                backLeft.getModuleState().angle.getDegrees() + 90, -backLeft.getModuleState().speedMetersPerSecond,
                backRight.getModuleState().angle.getDegrees() + 90, -backRight.getModuleState().speedMetersPerSecond
        });
    }

    public void zeroHeading() {
        setHeading(0);
    }

    public void setHeading(double deg) {
        if (Robot.isSimulation()) {
            navxSim = Units.degreesToRadians(deg);
        }
        navX.reset();
        navX.setAngleAdjustment(deg);
    }

    public Pose2d getPose() {
        Pose2d p = odometry.getEstimatedPosition();
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Robot.isSimulation() ? navxSim : Units.degreesToRadians(Math.IEEEremainder(-navX.getAngle(), 360));
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        lastChassisSpeeds = DriveConstants.KINEMATICS.toChassisSpeeds(states);
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(states[Constants.DriveConstants.ModuleIndices.FRONT_LEFT]);
        frontRight.setModuleState(states[Constants.DriveConstants.ModuleIndices.FRONT_RIGHT]);
        backRight.setModuleState(states[Constants.DriveConstants.ModuleIndices.REAR_RIGHT]);
        backLeft.setModuleState(states[Constants.DriveConstants.ModuleIndices.REAR_LEFT]);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = -speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = -speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp;
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setXstance() {
        // frontLeft.setModuleStateRaw(new SwerveModuleState(0,
        // Rotation2d.fromDegrees(45)));
        // frontRight.setModuleStateRaw(new SwerveModuleState(0,
        // Rotation2d.fromDegrees(-45)));
        // backLeft.setModuleStateRaw(new SwerveModuleState(0,
        // Rotation2d.fromDegrees(-45)));
        // backRight.setModuleStateRaw(new SwerveModuleState(0,
        // Rotation2d.fromDegrees(45)));
        frontLeft.setModuleStateRaw(new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)));
        backRight.setModuleStateRaw(new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DriveConstants.KINEMATICS.toChassisSpeeds(
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState());

        return speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = {
                frontLeft.getModulePosition(),
                frontRight.getModulePosition(),
                backLeft.getModulePosition(),
                backRight.getModulePosition()
        };

        return states;
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulate_step();
        frontRight.simulate_step();
        backLeft.simulate_step();
        backRight.simulate_step();
        navxSim += 0.02 * lastChassisSpeeds.omegaRadiansPerSecond;
    }

    public AprilTag getAprilTag(AprilTagPosition tagPosition, AprilTagType tagType)
    {
      // if there is an alliance it gets the alliance (blue or red)
      Optional<Alliance> alliance = DriverStation.getAlliance();
      AprilTag returnValue = null;
      // loops through the hashtable and finds the correct apriltag and returns the details
      if(alliance.isPresent()){
          Enumeration<String> e = Constants.AllAprilTags.keys();
          AprilTag tag = null;
          while(e.hasMoreElements()) {
              String key = e.nextElement();
              tag = Constants.AllAprilTags.get(key);
              if(tag != null && tag.GetAlliance() == alliance.get() && tag.GetTagPosition() == tagPosition && tag.GetTagType() == tagType){
                  returnValue = tag;
                  break;
              }
          }
      }
      return returnValue;
  }

  public AprilTag getAprilTagByID (String tagId){
   return Constants.AllAprilTags.get(tagId);
  }
}
