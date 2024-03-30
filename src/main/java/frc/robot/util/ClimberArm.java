package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ClimberConstants;

public class ClimberArm {
    public final CANSparkMax motor;

    private final Servo brake;
    private SlewRateLimiter motlim = new SlewRateLimiter(5.0);
    private double throttle = 0.0f;

    private final boolean isinverted;
    private final boolean ismirrored;
    final int motorCANID;

    public boolean is_calibrated = false;

    private double brake_release_time = 0.0;
    private boolean brake_engaged = true;

    private DeployMode deployMode = DeployMode.None;

    public enum DeployMode {
        None(1.5),
        Extend(3.7),
        FlipUp(6.3);

        double extensionLimitRots;

        DeployMode(double extl) {
            extensionLimitRots = extl;
        }
    }

    public ClimberArm(int canID, int brakeServoPort, boolean mirrored, boolean inverted) {
        motorCANID = canID;
        isinverted = inverted;
        ismirrored = mirrored;
        motor = new CANSparkMax(motorCANID, MotorType.kBrushless);
        brake = new Servo(brakeServoPort);

        hardwareInit();
    }

    private final double POS_BOT_LIM = -0.45;

    public void hardwareInit() {
        is_calibrated = false;
        brake_release_time = Timer.getFPGATimestamp() - 1.5;
        disengageBrake();

        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(isinverted);
        motor.getEncoder().setPositionConversionFactor(ClimberConstants.CLIMBER_POS_CONV_FACTOR);
        // motor.setSoftLimit(SoftLimitDirection.kForward, (float)
        // DeployMode.FlipUp.extensionLimitRots);
        setDeployMode(DeployMode.Extend);
        motor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);

        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) POS_BOT_LIM);

        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public boolean isRetracted() {
        return motor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public void engageBrake() {
        brake.setAngle(
                ismirrored ? 180 - ClimberConstants.BRAKE_ENGAGE_POSITION : ClimberConstants.BRAKE_ENGAGE_POSITION);
        brake_engaged = true;
    }

    public void disengageBrake() {
        brake.setAngle(ismirrored ? 180 - ClimberConstants.BRAKE_DISENGAGE_POSITION
                : ClimberConstants.BRAKE_DISENGAGE_POSITION);
        if (brake_engaged) {
            brake_release_time = Timer.getFPGATimestamp();
        }
        brake_engaged = false;
    }

    public boolean isSafeToMove() {
        return (!brake_engaged)
                && ((Timer.getFPGATimestamp() - brake_release_time) > ClimberConstants.BRAKE_DISENGAGE_TIME);
    }

    public void periodic() {
        if (isRetracted()) {
            if (!is_calibrated) {
                motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                motor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
                motor.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);

                motor.getEncoder().setPosition(0.0);
                engageBrake();
            }
            is_calibrated = true;
        }

        if (is_calibrated) {
            // || ((throttle < 0.01) && (motor.getEncoder().getPosition() < POS_BOT_LIM))
            if (Math.abs(throttle) < 0.01) {
                engageBrake();
            } else {
                disengageBrake();
            }

            double tout = isSafeToMove() ? throttle : 0.0;
            double tramp = motlim.calculate(tout);

            if (isSafeToMove()) {
                // TODO: For production bot
                motor.set(tramp); // motlim.calculate(throttle));
            } else {
                motor.stopMotor();
            }
        } else {
            disengageBrake();
            if (isSafeToMove())
                motor.set(-0.4);
        }
    }

    public void set(double power) {
        throttle = power;
    }

    public void setDeployMode(DeployMode mode) {
        if (mode != deployMode) {
            motor.setSoftLimit(SoftLimitDirection.kForward, (float) mode.extensionLimitRots);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        }
        deployMode = mode;
    }

    public DeployMode getDeployMode() {
        return deployMode;
    }

    public void stop() {
        motlim.reset(0.0f);
        throttle = 0.0f;
        motor.stopMotor();
    }
}
