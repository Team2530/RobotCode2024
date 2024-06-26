package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        REVERSE(-1.0),        
        ALIGN(-1.0),

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
        hardwareInit();
    }

    public void hardwareInit() {
        // var clc = new CurrentLimitsConfigs();
        // clc.StatorCurrentLimitEnable = false;
        // clc.SupplyCurrentLimitEnable = false;
        // shooterMotor.getConfigurator().apply(clc);
        // shooterMotor.setSafetyEnabled(false);

        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        shooterVelocityControl.Slot = 0;
        shooterSlot0.kV = ArmConstants.SHOOTER_kV;
        shooterSlot0.kP = ArmConstants.SHOOTER_kP;
        shooterSlot0.kI = ArmConstants.SHOOTER_kI;
        shooterSlot0.kD = ArmConstants.SHOOTER_kD;

        shooterMotor.getConfigurator().apply(shooterSlot0, 0.050);
    }

    public void stop() {
        shooterMotor.set(0);
        brake();
    }
    

    @Override
    public void periodic() {
        // double percent = (targetRPS) / ArmConstants.SHOOTER_MAX_RPS;
        // shooterMotor.setVoltage(percent * 12.0);



        if (shooterMode == ShooterMode.STOPPED || Math.abs(targetRPS) < 1.5) {
            shooterMotor.set(0);
        } else if (shooterMode == ShooterMode.ALIGN){
            shooterMotor.set(shooterMode.modeSpeed);
        } else {
            if (shooterMode == ShooterMode.REVERSE) {
                shooterMotor.set(-1.0);
            } else {
                shooterMotor.setControl(shooterVelocityControl.withSlot(0).withVelocity(targetRPS));
            }
        }

        SmartDashboard.putNumber("Shooter TARGET velocity", targetRPS);
        SmartDashboard.putNumber("Shooter REAL velocity", shooterMotor.getVelocity().getValueAsDouble());

        // SmartDashboard.putNumber("Shooter Percent", targetRPS);
        // SmartDashboard.putNumber("Shooter Real", shooterMotor.getRotorVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter INPUT", shooterMotor.getClosedLoopOutput().getValueAsDouble());

        // SmartDashboard.putString("Shootake", "Shooter mode set to " +
        // (shooterMode.name()));

        SmartDashboard.putNumberArray("Shooter Debug", new double[] {
            targetRPS,
            shooterMotor.getVelocity().getValueAsDouble()
        });
    }

    /**
     * Sets shooter RPM based on percent from Mode speed
     * 
     * @param mode
     */
    public void setMode(ShooterMode mode) {
        if (shooterMode == mode) {
            return;
        }
        
        shooterMode = mode;
        targetRPS = shooterMode.modeSpeed * ArmConstants.SHOOTER_MAX_RPS;
        if (shooterMode == ShooterMode.STOPPED || Math.abs(targetRPS) < 1.5) {
            brake();
        } else {
            coast();
        }
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

    public boolean isReadySpooled() {
        return (Math.abs(shooterMotor.getVelocity().getValueAsDouble() - targetRPS) < 5.0)
                && shooterMode != ShooterMode.STOPPED && (targetRPS > 1.0);
    }

    public boolean isStopped() {
        return Math.abs(shooterMotor.getVelocity().getValueAsDouble()) < 20.0;
    }

    public ShooterMode getShooterMode() {
        return shooterMode;
    }
}
