package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX left_firstJointMotor = new TalonFX(ArmConstants.LEFT_FIRST_MOTOR_ID);
    private final TalonFX right_firstJointMotor = new TalonFX(ArmConstants.RIGHT_FIRST_MOTOR_ID);

    private final TalonFX secondJointMotor = new TalonFX(ArmConstants.SECOND_MOTOR_ID);

    private final CANcoder firstJointEncoder = new CANcoder(ArmConstants.FIRST_ABSOLUTE_ENCODER_PORT);
    private final CANcoder secondJointEncoder = new CANcoder(ArmConstants.SECOND_ABSOLUTE_ENCODER_PORT);

    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(ArmConstants.SHOULDER_kS, ArmConstants.SHOULDER_kG, ArmConstants.SHOULDER_kV, ArmConstants.SHOULDER_kA);
    private final ProfiledPIDController shoulderFeedback = new ProfiledPIDController(ArmConstants.SHOULDER_kP, ArmConstants.SHOULDER_kI, ArmConstants.SHOULDER_kD, ArmConstants.SHOULDER_CONSTRAINTS);

    private final ArmFeedforward wristFeedforward = new ArmFeedforward(ArmConstants.WRIST_kS, ArmConstants.WRIST_kG, ArmConstants.WRIST_kV, ArmConstants.WRIST_kA);
    private final ProfiledPIDController wristFeedback = new ProfiledPIDController(ArmConstants.WRIST_kP, ArmConstants.WRIST_kI, ArmConstants.WRIST_kD, ArmConstants.WRIST_CONSTRAINTS);

    public ArmSubsystem () {
        left_firstJointMotor.setInverted(ArmConstants.LEFT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setInverted(ArmConstants.RIGHT_FIRST_MOTOR_REVERSED);

        secondJointMotor.setInverted(ArmConstants.SECOND_MOTOR_REVERSED);
    }

    @Override
    public void periodic () {

    }

    public void setWrist(double setpoint, double velocity) {
        double feedback = wristFeedback.calculate(secondJointEncoder.getPosition().getValue(), setpoint);
        double feedforward = wristFeedforward.calculate(setpoint, velocity);
        
        secondJointMotor.setVoltage(feedback + feedforward);
    }
}
