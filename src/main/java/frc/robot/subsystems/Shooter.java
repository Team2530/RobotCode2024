package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Shooter extends SubsystemBase {
    public enum ShooterMode {
        STOPPED(0.0),
        FULL(1.0),
        REVERSE(-0.2),
        IDLE(0.1),
        CUSTOM(1.5);

        private double modeSpeed;

        private ShooterMode(double modeSpeed) {
            this.modeSpeed = modeSpeed;
        }
    }

    // current wanted setpoint "mode" of the shooter
    private ShooterMode shooterMode = ShooterMode.STOPPED;

    // Falcon 500 shooter motor
    private final TalonFX shooterMotor = new TalonFX(ArmConstants.SHOOTER_MOTOR_PORT);

    private double outputPercent = 0.0;

    //TODO: TUNE! allow shooter to spool in 1/3 sec and stop in 1/2.
    private final SlewRateLimiter shooterProfile = new SlewRateLimiter(5, -5, 0.0);

    public Shooter() {
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        double percent = shooterProfile.calculate(outputPercent);
        shooterMotor.set(percent);

        SmartDashboard.putNumber("Shooter Percent", percent * 100);        
        SmartDashboard.putNumber("Shooter Real", shooterMotor.getRotorVelocity().getValueAsDouble());

    }

    public void setMode(ShooterMode mode) {
        shooterMode = mode;
        outputPercent = shooterMode.modeSpeed;

         SmartDashboard.putString("Shootake", "Shooter mode set to " + (shooterMode.name()));
    }

    public void setCustomPercent(double percent) {
        shooterMode = ShooterMode.CUSTOM;
        // clamp between (-1, 1)
        outputPercent = Math.max(-1, Math.min(percent, 1));

        SmartDashboard.putString("Shootake", "Shooter speed set to " + String.format("%.0f", percent * 100) + " percent");
    }

    public double getOutputPercent() {
        return outputPercent;
    }

    /**
     * Sets the shooter motor into coast mode
     */
    public void coast() {
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets the shooter motor into brake mode
     */
    public void brake() {
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean isUpToSpeed() {
        return shooterMotor.getRotorVelocity().getValueAsDouble() > (outputPercent * 90.0);
    }
}
