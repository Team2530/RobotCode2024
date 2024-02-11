package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem {
    private final TalonFX shooterMotor = new TalonFX(SHOOTER_ID);
    private final TalonFX intakeMotor = new TalonFX(INTAKE_ID);
    
    public IntakeSubsystem() {
        shooterMotor.setInverted(SHOOTER_REVERSED);
        intakeMotor.setInverted(INTAKE_REVERSED);
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
