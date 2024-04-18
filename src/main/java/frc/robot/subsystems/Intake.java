package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.BooleanSupplier;

import javax.swing.text.Utilities;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {

    public enum IntakeMode {
        STOPPED(0.0),
        INTAKING(1.0),        
        ALIGN(0.65),

        REVERSE(-0.1),
        SHOOT(0.75),
        CUSTOM(1.5);

        private double modeSpeed;

        private IntakeMode(double modeSpeed) {
            this.modeSpeed = modeSpeed;
        }
    }

    // Current Wanted setpoint of intake
    private IntakeMode intakeMode = IntakeMode.STOPPED;

    // Falcon 500 intake motor
    private final TalonFX intakeMotor = new TalonFX(ArmConstants.INTAKE_MOTOR_PORT);
    HardwareLimitSwitchConfigs limconf = new HardwareLimitSwitchConfigs();

    // desired custom motor output percent
    private double outputPercent = 0.0;

    // private final PIDController intakeProfile = new PIDController(0.1, 0.0,
    // 0.01);

    // allow motor to speed up quickly and slow down over a period of time
    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(5, -5, 0.0);

    private final CommandXboxController driverXboxController;
    private final CommandXboxController operatorXboxController;

    public Intake(CommandXboxController driverXboxController, CommandXboxController operatXboxController) {
        hardwareInit();

        this.driverXboxController = driverXboxController;
        this.operatorXboxController = operatXboxController;
    }

    public void hardwareInit() {
        limconf = new HardwareLimitSwitchConfigs();
        limconf.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        limconf.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        limconf.ReverseLimitEnable = false;
        limconf.ForwardLimitEnable = true;
        intakeMotor.getConfigurator().apply(limconf);

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        double percent = intakeProfile.calculate(outputPercent);
        if (intakeMode == IntakeMode.INTAKING) {
            percent *= getIntakeSideLimitClosed() ? 0.5f : 1.0f;
        }
        intakeMotor.set(percent);

        SmartDashboard.putNumber("Intake Percent", percent * 100);
        SmartDashboard.putBoolean("Intake FWD Limit", getShooterSideLimitClosed());
        SmartDashboard.putBoolean("Intake REV Limit", getIntakeSideLimitClosed());

        if (getIntakeSideLimitClosed() && DriverStation.isTeleop() == true && DriverStation.isDisabled() == false) {
            driverXboxController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
            operatorXboxController.getHID().setRumble(RumbleType.kBothRumble, 0.5);

        } else {
            driverXboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
            operatorXboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        }

    }

    public BooleanSupplier containsNote() {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getIntakeSideLimitClosed() || getShooterSideLimitClosed();
            }
        };
    }

    /**
     * Sets intake mode
     * 
     * @param mode {@link IntakeMode} desired intake mode
     */
    public void setMode(IntakeMode mode) {
        intakeMode = mode;
        outputPercent = intakeMode.modeSpeed;

        SmartDashboard.putString("Shootake", "Intake mode set to " + (intakeMode.name()));
    }

    public double getIntakePosition() {
        return intakeMotor.getPosition().getValueAsDouble() * ArmConstants.INTAKE_ENCODER_TO_ROT;
    }

    /**
     * Changes the intake state to custom, and sets motor to custom output
     * 
     * @param percent output percent from (-1 to 1)
     */
    public void setCustomPercent(double percent) {
        intakeMode = IntakeMode.CUSTOM;
        // clamp between (-1,1)
        outputPercent = Math.max(-1, Math.min(percent, 1));

        SmartDashboard.putString("Shootake",
                "Intake speed set to " + String.format("%.0f", percent * 100) + " percent");
    }

    public double getOutputPercent() {
        return outputPercent;
    }

    public void coast() {
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // Has note in trap
    public boolean getShooterSideLimitClosed() {
        return intakeMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getIntakeSideLimitClosed() {
        return intakeMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public void setShooterLimitEnabled(boolean enabled) {
        limconf.ForwardLimitEnable = enabled;
        intakeMotor.getConfigurator().apply(limconf);
    }
}
