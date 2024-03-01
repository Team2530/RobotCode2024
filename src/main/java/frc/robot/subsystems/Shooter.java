package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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
    VelocityVoltage shooterVelocityControl = new VelocityVoltage(0);
    Slot0Configs shooterSlot0 = new Slot0Configs();

    private double targetRPS = 0.0;

    public Shooter() {
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        shooterVelocityControl.Slot = 0;
        shooterSlot0.kV = ArmConstants.SHOOTER_kV;
        shooterSlot0.kP = ArmConstants.SHOOTER_kP;
        shooterSlot0.kI = ArmConstants.SHOOTER_kI;
        shooterSlot0.kD = ArmConstants.SHOOTER_kD;

        shooterMotor.getConfigurator().apply(shooterSlot0, 0.050);
    }

    @Override
    public void periodic() {
        // double percent = (targetRPS) / ArmConstants.SHOOTER_MAX_RPS;
        // shooterMotor.setVoltage(percent * 12.0);

        if (targetRPS < 1.5) {
            shooterMotor.set(0);
        } else {
            shooterMotor.setControl(shooterVelocityControl.withSlot(0).withVelocity(targetRPS));
        }

        SmartDashboard.putNumber("Shooter TARGET velocity", targetRPS);
        SmartDashboard.putNumber("Shooter REAL velocity", shooterMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Shooter Percent", targetRPS);
        SmartDashboard.putNumber("Shooter Real", shooterMotor.getRotorVelocity().getValueAsDouble());
        // SmartDashboard.putString("Shootake", "Shooter mode set to " +
        // (shooterMode.name()));
    }

    /**
     * Sets shooter RPM based on percent from Mode speed
     * 
     * @param mode
     */
    public void setMode(ShooterMode mode) {
        shooterMode = mode;
        targetRPS = shooterMode.modeSpeed * ArmConstants.SHOOTER_MAX_RPS;
        SmartDashboard.putNumber("Shootake", mode.modeSpeed);
    }

    public void setCustomPercent(double percent) {
        shooterMode = ShooterMode.CUSTOM;
        // clamp between (-1, 1)

        targetRPS = MathUtil.clamp(percent, -1.0, 1.0);// Math.max(-1, Math.min(percent, 1));
        targetRPS *= ArmConstants.SHOOTER_MAX_RPS;

        SmartDashboard.putString("Shootake",
                "Shooter speed set to " + String.format("%.0f", percent * 100) + " percent");
    }

    /**
     * Gets output percent of shooter, <em>not RPM!</em>
     * 
     * @return output percent
     */
    public double getTargetRPS() {
        return targetRPS;
    }

    /**
     * Gets the output RPM of the motor
     * 
     * @return
     */
    public double getOutputRPM() {
        return targetRPS;// * ArmConstants.SHOOTER_MAX_RPS;
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
        return shooterMotor.getVelocity().getValueAsDouble() > (targetRPS * .9);
    }
}
