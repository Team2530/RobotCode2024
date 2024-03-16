package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.ClimberConstants;

public class ClimberArm {
    public CANSparkMax motor;
    SlewRateLimiter motlim = new SlewRateLimiter(5.0);
    double throttle = 0.0f;

    final boolean isinverted;
    final int motorCANID;

    public boolean is_calibrated = false;

    public ClimberArm(int canID, boolean inverted) {
        motorCANID = canID;
        isinverted = inverted;
        motor = new CANSparkMax(motorCANID, MotorType.kBrushless);

        hardwareInit();
    }

    public void hardwareInit() {
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(isinverted);
        motor.getEncoder().setPositionConversionFactor(ClimberConstants.CLIMBER_POS_CONV_FACTOR);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) ClimberConstants.CLIMBER_LENGTH);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    public boolean isRetracted() {
        return motor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public void periodic() {
        if (isRetracted()) {
            motor.getEncoder().setPosition(0.0);
            is_calibrated = true;
        }

        // TODO: For production bot
        if (is_calibrated) {
            motor.set(motlim.calculate(throttle));
        } else {
            motor.set(-0.4);
        }
    }

    public void set(double power) {
        throttle = power;
    }

    public void stop() {
        motlim.reset(0.0f);
        throttle = 0.0f;
        motor.stopMotor();
    }
}
