// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81;
    public static final double SPEAKER_HEIGHT = 2.05; // Meters

    public static Translation2d getSpeakerPosition() {
      Translation2d speakerBlue = new Translation2d(0.022, 5.55);
      speakerBlue = getAlliance() == Alliance.Blue ? speakerBlue
          : GeometryUtil.flipFieldPosition(speakerBlue);
      return speakerBlue;
    }

    public static Alliance getAlliance() {
      if(DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get();
      }

      return Alliance.Blue;
    }
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
    public static final double MODULE_KP = 0.56368;// 0.75628;// 0.7491; //.5;
    public static final double MODULE_KD = 0.0066806;// 0.0057682; //0.0076954;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 1;
    public static final int FL_STEER_ID = 2;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(0.213135) + Math.PI * 0.5 + Math.PI;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FL_MOTOR_REVERSED = true;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 7;
    public static final int FR_STEER_ID = 8;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 2;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(-0.305908) + Math.PI * 0.5 + Math.PI;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FR_MOTOR_REVERSED = true;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 5;
    public static final int BR_STEER_ID = 6;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(0.230225) + Math.PI * 0.5 + Math.PI;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BR_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 3;
    public static final int BL_STEER_ID = 4;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 4;
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(-0.077637) + Math.PI * 0.5 + Math.PI;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BL_MOTOR_REVERSED = true;

  }

  public static class DriveConstants {
    public static final double MAX_MODULE_VELOCITY = 4.8;
    public static final double MAX_ROBOT_VELOCITY = 4.8;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    // TODO: Change based on actual robot!
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(24.75);
    public static final Rotation2d NAVX_ANGLE_OFFSET = Rotation2d.fromDegrees(90);
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(15);

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
    public static final boolean LOG_INTO_FILE_ENABLED = false;
  }

  public static class VisionContsants {

    public static final double THETA_kP = 5;
    public static final double THETA_kI = 0.0;
    public static final double THETA_kD = 0.08;

    public static final double X_kP = 1.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.02;

    public static final double Y_kP = 5;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.02;
  }

  public static class LimelightConstants {
    public static final String limeLightName = "limelight";
    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.06, -0.2, 0.2127),
        new Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(3.0)));
    public static final boolean LOG_APRIL_TAGS_INTO_SMARTDASH_BOARD = false;
  }

  public static class AprilTags {
    // Blue alliance left or single tags
    public static final String[] BLUE_ALLIANCE_LEFT_OR_SINGLE_APRILTAGS = { "2", "8", "6", "14", "15", "16" };
    // Blue alliance right tags
    public static final String[] BLUE_ALLIANCE_RIGHT_APRILTAGS = { "1", "7" };
    // Red alliance left or single tags
    public static final String[] RED_ALLIANCE_LEFT_OR_SINLGE_APRILTAGS = { "10", "4", "5", "11", "12", "13" };
    // Blue alliance right tags
    public static final String[] RED_ALLIANCE_RIGHT_APRILTAGS = { "9", "3" };
  }

  public static final class PathPlannerConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0.2);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0.2);

    public static final HolonomicPathFollowerConfig HOLONOMIC_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        TRANSLATION_PID,
        ROTATION_PID,
        DriveConstants.MAX_MODULE_VELOCITY,
        DriveConstants.DRIVE_BASE_RADIUS,
        new ReplanningConfig());
  }

  public static final class ArmConstants {

    public static final int STAGE_ONE_MOTOR_L = 4;

    public static final int STAGE_ONE_MOTOR_R = 3;

    public static final int STAGE_TWO_MOTOR_PORT = 5;

    public static final int STAGE_ONE_ENCODER_PORT = 11;

    public static final int INTAKE_MOTOR_PORT = 1;

    public static final int SHOOTER_MOTOR_PORT = 2;

    public static final int STAGE_TWO_ENCODER_PORT = 10;

    // Link lengths in inches
    public static final double STAGE_ONE_LENGTH = 19.7;
    public static final double STAGE_TWO_LENGTH = 12.0;

    public static final String STAGE_ONE_OFFSET_KEY = "STAGE_ONE_OFFSET";
    public static final String STAGE_TWO_OFFSET_KEY = "STAGE_TWO_OFFSET";

    public static final double STAGE_ONE_ENCODER_OFFSET = 0.0175;

    public static final double STAGE_TWO_ENCODER_OFFSET = -0.394043 - 0.066;

    public static final double INTAKE_ENCODER_TO_ROT = 10.0 / 18.0;

    public static final ProfiledPIDController STAGE_ONE_PROFILEDPID = new ProfiledPIDController(
        3,
        0.0,
        0.0,
        new Constraints(3, 3));

    public static final ProfiledPIDController STAGE_TWO_PROFILEDPID = new ProfiledPIDController(
        12,
        0.0,
        0.001,
        new Constraints(12, 12));

    public static final ArmFeedforward STAGE_ONE_FEEDFORWARD = new ArmFeedforward(
        0.0,
        0.18,
        2.07,
        0.02);

    public static final ArmFeedforward STAGE_TWO_FEEDFORWARD = new ArmFeedforward(
        0.0,
        0.26,
        1.19,
        0.0);

    public static final boolean L_STAGE_ONE_ISREVERSED = true;
    public static final boolean FOLLOWER_STAGE_ONE_ISREVERSED = true;
    public static final boolean STAGE_TWO_ISREVERSED = true;

    public static final boolean STAGE_ONE_ENCODER_ISREVERSED = true;

    public static final boolean STAGE_TWO_ENCODER_ISREVERSED = false;

    public static final double HUMAN_ARM_INPUT_P = 0.0001;

    public static final double SHOOTER_kV = 0.104;
    public static final double SHOOTER_kP = 0.2;
    public static final double SHOOTER_kI = 0.48;
    public static final double SHOOTER_kD = 0.01;
    public static final double SHOOTER_MAX_RPS = 106.0;

    // FOR TARGETING!!!
    public static final double MAX_SHOOTER_VELOCITY = 21.0;// 16.6;

    public static final double SHOOTER_LOW_HEIGHT = 0.7;
    public static final double SHOOTER_LOW_X_OFFSET = 0.36;

    public static final double SHOOTER_HIGH_HEIGHT = 0.96;
    public static final double SHOOTER_HIGH_X_OFFSET = -0.25;

  }

  public static class ClimberConstants {
    public static final int LEFT_CLIMBER_CANID = 20;
    public static final int RIGHT_CLIMBER_CANID = 30;

    public static final double ROLL_kP = 15.0; // 1.0 is full side rotation;
    public static final double GOOD_THRESHOLD = 10.0;// Degrees!

    public static final boolean LEFT_CLIMBER_INVERTED = false;
    public static final boolean RIGHT_CLIMBER_INVERTED = true;

    public static final double CLIMBER_LENGTH = 11.2; // Inches-ish
    public static final double SPOOL_RADIUS = 0.5; // CHECK WITH SPIRAL SPOOLING!
    public static final double SPOOL_CIRC = 2 * Math.PI * SPOOL_RADIUS;

    public static final double CLIMBER_RATIO = 1.0 / 100.0;

    public static final double CLIMBER_POS_CONV_FACTOR = SPOOL_CIRC * CLIMBER_RATIO;
  }

  public static final class PoseConstants {

    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevTheta = 500;
  }
}
