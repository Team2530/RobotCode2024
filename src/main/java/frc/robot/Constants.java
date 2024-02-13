// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

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

  public static final class ArmConstants {

    // TODO: Update constants for real robot, are currently probably outdated!
    // All arm constants to do with placement are refrenced from intake side and bottom plate.
    // NOTE: All units are in inches
    public static final Translation2d MAST_CENTER_OF_ROTATION = new Translation2d(-9.6, 22.780);

    public static final int STAGE_ONE_MOTOR_L = 4;    

    public static final int STAGE_ONE_MOTOR_R = 3;

    public static final int STAGE_TWO_MOTOR_PORT = 11;

    public static final int STAGE_ONE_ENCODER_PORT = 11;

    public static final int INTAKE_MOTOR_PORT = 1;

    public static final int SHOOTER_MOTOR_PORT = 2;

    public static final int STAGE_TWO_ENCODER_PORT = 10;
    
    // Link lengths in inches
    public static final double STAGE_ONE_LENGTH = 19.7;
    public static final double STAGE_TWO_LENGTH = 12.0;

    public static final String STAGE_ONE_OFFSET_KEY = "STAGE_ONE_OFFSET";
    public static final String STAGE_TWO_OFFSET_KEY = "STAGE_TWO_OFFSET";

    public static final double STAGE_ONE_ENCODER_OFFSET = Units.rotationsToRadians(0.0);

    public static final double STAGE_TWO_ENCODER_OFFSET = Units.rotationsToRadians(0.0);

    public static final double INTAKE_ENCODER_TO_ROT = 10.0 / 18.0;

    public static final int MAST_ARM_ENCODER_PORT = 5;

    public static final ProfiledPIDController STAGE_ONE_PROFILEDPID = new ProfiledPIDController(
      0.1, 
      0.0, 
      0.0, 
      new Constraints(1, 1));

      public static final ProfiledPIDController STAGE_TWO_PROFILEDPID = new ProfiledPIDController(
      0.1, 
      0.0,
      0.0, 
      new Constraints(1, 1));

    public static final ArmFeedforward STAGE_ONE_FEEDFORWARD = new ArmFeedforward(
      0.0,
      0.42, 
      2.07,
      0.02);

    public static final ArmFeedforward STAGE_TWO_FEEDFORWARD = new ArmFeedforward(
      0.0,
      0.26, 
      1.19,
      0.0);

    public static final boolean L_STAGE_ONE_ISREVERSED = false;
    public static final boolean R_STAGE_ONE_ISREVERSED = false;









    
  }
}
