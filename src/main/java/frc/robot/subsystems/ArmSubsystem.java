package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import static java.lang.Math.*;

public class ArmSubsystem extends SubsystemBase {
    // -------- First Joint ------- \\
    private final TalonFX left_firstJointMotor = new TalonFX(ArmConstants.LEFT_FIRST_MOTOR_ID);
    private final TalonFX right_firstJointMotor = new TalonFX(ArmConstants.RIGHT_FIRST_MOTOR_ID);
    
    private final CANcoder firstJointEncoder = new CANcoder(ArmConstants.FIRST_ABSOLUTE_ENCODER_PORT);

    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(ArmConstants.SHOULDER_kS, ArmConstants.SHOULDER_kG, ArmConstants.SHOULDER_kV, ArmConstants.SHOULDER_kA);
    private final ProfiledPIDController shoulderFeedback = new ProfiledPIDController(ArmConstants.SHOULDER_kP, ArmConstants.SHOULDER_kI, ArmConstants.SHOULDER_kD, ArmConstants.SHOULDER_CONSTRAINTS);


    // -------- Second Joint ------ \\
    private final TalonFX secondJointMotor = new TalonFX(ArmConstants.SECOND_MOTOR_ID);

    private final CANcoder secondJointEncoder = new CANcoder(ArmConstants.SECOND_ABSOLUTE_ENCODER_PORT);

    private final ArmFeedforward wristFeedforward = new ArmFeedforward(ArmConstants.WRIST_kS, ArmConstants.WRIST_kG, ArmConstants.WRIST_kV, ArmConstants.WRIST_kA);
    private final ProfiledPIDController wristFeedback = new ProfiledPIDController(ArmConstants.WRIST_kP, ArmConstants.WRIST_kI, ArmConstants.WRIST_kD, ArmConstants.WRIST_CONSTRAINTS);


    public ArmSubsystem () {
        left_firstJointMotor.setInverted(ArmConstants.LEFT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setInverted(ArmConstants.RIGHT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setControl(new Follower(left_firstJointMotor.getDeviceID(), true));

        secondJointMotor.setInverted(ArmConstants.SECOND_MOTOR_REVERSED);
    }

    public void setWrist(double setpoint, double velocity) {
        double feedback = wristFeedback.calculate(secondJointEncoder.getPosition().getValue(), setpoint);
        double feedforward = wristFeedforward.calculate(setpoint, velocity);
        
        secondJointMotor.setVoltage(feedback + feedforward);
    }

    public void setShoulder(double setpoint, double velocity) {
        double feedback = shoulderFeedback.calculate(firstJointEncoder.getPosition().getValue(), setpoint);
        double feedforward = shoulderFeedforward.calculate(setpoint, velocity);

        left_firstJointMotor.setVoltage(feedback + feedforward);
    }

    public Translation2d getCurrentEndpoint() {
        final double length1 = Constants.ArmConstants.L1;
        final double length2 = Constants.ArmConstants.L2;

        final double angle1 = firstJointEncoder.getPosition().getValueAsDouble();
        final double angle2 = secondJointEncoder.getPosition().getValueAsDouble();

        final Translation2d a = new Translation2d(sin(angle1), -cos(angle1)).times(length1);
        final Translation2d b = new Translation2d(sin(angle1 + angle2), -cos(angle1 + angle2)).times(length2);

        return a.plus(b);
    }
}
