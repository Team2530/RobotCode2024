// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;
  }

  public static class SwerveModuleConstants {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = 1.d / (150d / 7d);
    public static final double DRIVE_GEAR_RATIO = 1.d / 6.75d;

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;

    // Actual drive gains
    // public static final double MODULE_KP = 0.5;
    // public static final double MODULE_KD = 0.03;

    // NOTE: This may need additional tuning!
    public static final double MODULE_KP = 0.5;// 0.75628;// 0.7491; //0.56368;
    public static final double MODULE_KD = 0.0066806;// 0.0057682; //0.0076954;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 2;
    public static final int FL_STEER_ID = 1;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(0.074951) + Math.PI * 0.5;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FL_MOTOR_REVERSED = false;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 4;
    public static final int FR_STEER_ID = 3;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 4;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(-0.062256) + Math.PI * 0.5;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FR_MOTOR_REVERSED = false;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 6;
    public static final int BR_STEER_ID = 5;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 2;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(0.152100) + Math.PI * 0.5;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BR_MOTOR_REVERSED = false;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 8;
    public static final int BL_STEER_ID = 7;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(-0.109131) + Math.PI * 0.5;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BL_MOTOR_REVERSED = false;

  }

  public static class DriveConstants {
    public static final double MAX_MODULE_VELOCITY = 4.8;
    public static final double MAX_ROBOT_VELOCITY = 4.8;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    // TODO: Change based on actual robot!
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
    public static final Rotation2d NAVX_ANGLE_OFFSET = Rotation2d.fromDegrees(90);

    public static final class ModuleIndices {
      public static final int FRONT_LEFT = 0;
      public static final int FRONT_RIGHT = 2;
      public static final int REAR_LEFT = 1;
      public static final int REAR_RIGHT = 3;
    }

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 1.0;
    public static final double Z_SPEED_LIMIT = 1.0;
  }

  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = true;
  }

  public static class VisionContsants {
    
    public static final double THETA_kP = .9;
    public static final double THETA_kI = 0.0;
    public static final double THETA_kD = 0.08;

    public static final double X_kP = 1.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.02;

    public static final double Y_kP = 1.5;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.02;
  }

  public static class LimelightConstants {
    public static final String limeLightName = "limelight";
    public static final Transform3d robotToCamera = new Transform3d(
    new Translation3d(0.06, -0.2, 0.2127),
    new Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(3.0)));
    public static final boolean LOG_APRIL_TAGS_INTO_SMARTDASH_BOARD = true;
    public static final int CLEAR_APRILTAG_INTERVAL = 40; // in mili sec
    public static final int APRILTAG_SEARCH_ROTATION = 40; // in degrees
    public static Constraints pidXConstriants = new Constraints(5, 5);
    public static Constraints pidYConstraints = new Constraints(5, 5);
    public static Constraints pidOmegaConstraints = new Constraints(20, 20);
  }

  public static enum AprilTagPosition {
    LEFT,
    RIGHT,
    CENTER
  }

  public static enum AprilTagType {
    SOURCE,
    AMP,
    SPEAKER,
    STAGE,
  }

  public static Dictionary<String, AprilTag> AllAprilTags = new Hashtable<String, AprilTag>() {{
    //Blue Alliance April tags 1, 2, 6, 7, 8, 14, 15, 16
    //Red Alliance April tags 3, 4, 5, 9, 10, 11, 12, 13
    put("1.0", new AprilTag("1.0", AprilTagType.SOURCE, AprilTagPosition.RIGHT, Alliance.Blue, 593.68, 9.68, 53.38, 120.00, 25.0, 0, 0));
    put("2.0", new AprilTag("2.0", AprilTagType.SOURCE, AprilTagPosition.LEFT, Alliance.Blue, 637.21, 34.79, 53.38, 120.00, 25.0, 0, 0));
    put("3.0", new AprilTag("3.0", AprilTagType.SPEAKER, AprilTagPosition.RIGHT, Alliance.Red, 652.73,196.17, 57.13, 180.00, 55.0, 0, 30));
    put("4.0", new AprilTag("4.0", AprilTagType.SPEAKER, AprilTagPosition.LEFT, Alliance.Red, 652.73, 218.42, 57.13, 180.00, 55.0, 0, 0));
    put("5.0", new AprilTag("5.0", AprilTagType.AMP, AprilTagPosition.CENTER, Alliance.Red, 578.77, 323.00, 53.38, 270.00, 20.0, 0, 0));
    put("6.0", new AprilTag("6.0", AprilTagType.AMP, AprilTagPosition.CENTER, Alliance.Blue, 72.5, 323.00, 53.38, 270.00, 20, 0, 0));
    put("7.0", new AprilTag("7.0", AprilTagType.SPEAKER, AprilTagPosition.RIGHT, Alliance.Blue, -1.50, 218.42, 57.13, 0.00, 55, 0, 45));
    put("8.0", new AprilTag("8.0", AprilTagType.SPEAKER, AprilTagPosition.LEFT, Alliance.Blue, -1.50, 196.17, 57.13, 0.00, 55, 0, 0));
    put("9.0", new AprilTag("9.0", AprilTagType.SOURCE, AprilTagPosition.RIGHT, Alliance.Red, 14.02, 34.79, 53.38, 60.00, 25, 0, 0));
    put("10.0", new AprilTag("10.0", AprilTagType.SOURCE, AprilTagPosition.LEFT, Alliance.Red, 57.54, 9.68, 53.38, 60.00, 20, 0, 0));
    put("11.0", new AprilTag("11.0", AprilTagType.STAGE, AprilTagPosition.CENTER, Alliance.Red, 468.69, 146.19, 52.00, 300.00, 16, 0, 0));
    put("12.0", new AprilTag("12.0", AprilTagType.STAGE, AprilTagPosition.CENTER, Alliance.Red, 468.69, 177.10, 52.00, 60.00, 16, 0, 0));
    put("13.0", new AprilTag("13.0", AprilTagType.STAGE, AprilTagPosition.CENTER, Alliance.Red, 441.74, 161.62, 52.00, 180.00, 16, 0, 0));
    put("14.0", new AprilTag("14.0", AprilTagType.STAGE, AprilTagPosition.CENTER, Alliance.Blue, 209.48, 161.62, 52.00, 0.00, 16, 0, 0));
    put("15.0", new AprilTag("15.0", AprilTagType.STAGE, AprilTagPosition.CENTER, Alliance.Blue, 182.73, 177.10, 52.00, 120.00, 16, 0, 0));
    put("16.0", new AprilTag("16.0", AprilTagType.STAGE, AprilTagPosition.CENTER, Alliance.Blue, 182.73, 146.19, 52.00, 240.00, 16, 0, 0));
  }};
}