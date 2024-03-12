package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private double driveEncSim = 0;
    private double steerEncSim = 0;

    private final com.ctre.phoenix6.hardware.CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;

    private final PIDController steerPID;

    private static int moduleNumber = 0;
    int thisModuleNumber;

    SlewRateLimiter turnratelimiter = new SlewRateLimiter(4.d);

    DCMotor steerMotor_sim = DCMotor.getNEO(1);
    DCMotor driveMotor_sim = DCMotor.getNEO(1);

    FlywheelSim steerSim = new FlywheelSim(steerMotor_sim, 1.0 / SwerveModuleConstants.STEERING_GEAR_RATIO,
            SwerveModuleConstants.MODULE_ROTATIONAL_INERTIA);
    FlywheelSim driveSim = new FlywheelSim(driveMotor_sim, 1.0 / SwerveModuleConstants.DRIVE_GEAR_RATIO,
            SwerveModuleConstants.MODULE_DRIVE_SIM_INERTIA);

    public SwerveModule(
            int steerCanID,
            int driveCanID,
            int absoluteEncoderPort,
            double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed,
            boolean motorReversed) {
        driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        driveMotor.setInverted(motorReversed);
        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor = new CANSparkMax(steerCanID, MotorType.kBrushless);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(false);

        this.motor_inv = motorReversed;
        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        // Reset encoder offsets possibly set in Tuner X
        absoluteEncoder = new com.ctre.phoenix6.hardware.CANcoder(absoluteEncoderPort);
        com.ctre.phoenix6.configs.CANcoderConfiguration cfg = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        cfg.MagnetSensor = new com.ctre.phoenix6.configs.MagnetSensorConfigs();
        cfg.MagnetSensor.MagnetOffset = 0.0f;
        absoluteEncoder.getConfigurator().apply(cfg);
        // CANcoderConfigurator configurator = absoluteEncoder.getConfigurator();

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        driveMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER);
        driveMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);

        steerMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.STEER_ROTATION_TO_RADIANS);
        steerMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.STEER_RADIANS_PER_MINUTE);

        steerPID = new PIDController(SwerveModuleConstants.MODULE_KP, 0, SwerveModuleConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        thisModuleNumber = moduleNumber;
        moduleNumber++;

        resetEncoders();
    }

    public void simulate_step() {
        driveSim.setInput(driveMotor.get() * RobotController.getBatteryVoltage());
        driveSim.update(0.02);

        driveEncSim += 0.02 * driveSim.getAngularVelocityRadPerSec() * (SwerveModuleConstants.WHEEL_DIAMETER);

        steerSim.setInput(steerMotor.get() * RobotController.getBatteryVoltage());
        steerSim.update(0.02);

        steerEncSim += 0.02 * 2.0 * steerSim.getAngularVelocityRadPerSec();

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        driveSim.getCurrentDrawAmps() + steerSim.getCurrentDrawAmps()));

        // driveEncSim += 0.02 * driveMotor.get() *
        // (DriveConstants.MAX_MODULE_VELOCITY);
        // steerEncSim += 0.02 * steerMotor.get() * (50.0);
    }

    public double getDrivePosition() {
        if (Robot.isSimulation())
            return driveEncSim;
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    public double getSteerPosition() {
        if (Robot.isSimulation())
            return steerEncSim;
        return steerMotorEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getPosition().getValue());// * (Math.PI /
        // 180.d);
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),
                new Rotation2d(-getSteerPosition()).rotateBy(DriveConstants.NAVX_ANGLE_OFFSET.times(-1)));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(getSteerPosition()));
        double drive_command = state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY;
        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        // This is stupid
        // steerPID.setP(Constants.SwerveModuleConstants.MODULE_KP *
        // Math.abs(drive_command));
        double steercmd = steerPID.calculate(getSteerPosition(), state.angle.getRadians());
        if (Robot.isSimulation()) {
            steerMotor.set(steercmd);
        } else {
            steerMotor.setVoltage(12 * steercmd);
        }
        // SmartDashboard.putNumber("Abs" + thisModuleNumber,
        // getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
