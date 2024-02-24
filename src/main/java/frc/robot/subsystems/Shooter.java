package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    private double targetRPM = 0.0;

    public Shooter() {
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        double percent = (targetRPM) / ArmConstants.MAX_SHOOTER_RPM;
        shooterMotor.set(percent);

        SmartDashboard.putNumber("Shooter Percent", targetRPM);    
        SmartDashboard.putNumber("Shooter Real", shooterMotor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putString("Shootake", "Shooter mode set to " + (shooterMode.name()));

    }

    /**
     * Sets shooter RPM based on percent from Mode speed
     * @param mode
     */
    public void setMode(ShooterMode mode) {
        shooterMode = mode;
        targetRPM = shooterMode.modeSpeed * ArmConstants.MAX_SHOOTER_RPM;
        shooterProfile.setSetpoint(targetRPM);
    }

    public void setCustomPercent(double percent) {
        shooterMode = ShooterMode.CUSTOM;
        // clamp between (-1, 1)

        targetRPM = Math.max(-1, Math.min(percent, 1));

        targetRPM = targetRPM * ArmConstants.MAX_SHOOTER_RPM;
        shooterProfile.setSetpoint(targetRPM);

        SmartDashboard.putString("Shootake", "Shooter speed set to " + String.format("%.0f", percent * 100) + " percent");
    }

    /**
     * Gets output percent of shooter, <em>not RPM!</em>
     * @return output percent
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Gets the output RPM of the motor
     * @return
     */
    public double getOutputRPM() {
        return targetRPM * ArmConstants.MAX_SHOOTER_RPM;
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
        return shooterMotor.getRotorVelocity().getValueAsDouble() > (targetRPM * .9);
    }
}
