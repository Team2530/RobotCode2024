package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.*;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

    private SysIdRoutine DRIVE_SysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, SysIdRoutineLogger.logState()),
        new SysIdRoutine.Mechanism(this::setDriveMotorVoltages, null, this)
    );
    private SysIdRoutine STEER_SysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, SysIdRoutineLogger.logState()),
        new SysIdRoutine.Mechanism(this::setSteerMotorVoltages, null, this)
    );
      
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

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

        // TODO: Test
        // WARNING: REMOVE IF USING TAG FOLLOW!!!
        // odometry.addVisionMeasurement(LimelightHelpers.getBotPose2d(null),
        // Timer.getFPGATimestamp());

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

    public void setDriveMotorVoltages(Measure<Voltage> voltage) {
        frontLeft.setDriveVoltage(voltage);
        frontRight.setDriveVoltage(voltage);
        backLeft.setDriveVoltage(voltage);
        backRight.setDriveVoltage(voltage);
    }

    public void setSteerMotorVoltages(Measure<Voltage> voltage) {
        frontLeft.setSteerVoltage(voltage);
        frontRight.setSteerVoltage(voltage);
        backLeft.setSteerVoltage(voltage);
        backRight.setSteerVoltage(voltage);
    }

    public void sysIdDriveMotorLog(SysIdRoutineLog log){
        // Record a frame for the front left motors
         log.motor("drive-front-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeft.getDriveVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontLeft.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeft.getDriveVelocity(), MetersPerSecond));
        log.motor("drive-front-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRight.getDriveVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontRight.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontRight.getDriveVelocity(), MetersPerSecond));
        log.motor("drive-back-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backLeft.getDriveVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backLeft.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backLeft.getDriveVelocity(), MetersPerSecond));
        log.motor("drive-back-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRight.getDriveVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backRight.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backRight.getDriveVelocity(), MetersPerSecond));
    }

    public void sysIdSteerMotorLog(SysIdRoutineLog log){
        // Record a frame for the front left motors
         log.motor("steer-front-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeft.getSteerVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontLeft.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeft.getSteerVelocity(), MetersPerSecond));
        log.motor("steer-front-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRight.getSteerVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontRight.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontRight.getSteerVelocity(), MetersPerSecond));
        log.motor("steer-back-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backLeft.getSteerVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backLeft.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backLeft.getSteerVelocity(), MetersPerSecond));
        log.motor("steer-back-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRight.getSteerVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backRight.getAbsoluteEncoderPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backRight.getSteerVelocity(), MetersPerSecond));
    }


    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = -speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = -speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp;
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0,
        Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0,
        Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0,
        Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0,
        Rotation2d.fromDegrees(45)));
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

    public Command sysIdDriveQuasiCommand(Direction direction) {
        return DRIVE_SysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDriveDynamicCommand(Direction direction) {
        return DRIVE_SysIdRoutine.dynamic(direction);
    }

    public Command sysIdSteerQuasiCommand(Direction direction) {
        return STEER_SysIdRoutine.quasistatic(direction);
    }

    public Command sysIdSteerDynamicCommand(Direction direction) {
        return DRIVE_SysIdRoutine.dynamic(direction);
    }
}
