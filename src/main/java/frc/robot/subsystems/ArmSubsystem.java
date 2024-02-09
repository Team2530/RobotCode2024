package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static java.lang.Math.*;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
    // -------- First Joint ------- \\
    private final TalonFX left_firstJointMotor = new TalonFX(LEFT_FIRST_MOTOR_ID);
    private final TalonFX right_firstJointMotor = new TalonFX(RIGHT_FIRST_MOTOR_ID);
    
    private final CANcoder firstJointEncoder = new CANcoder(FIRST_ABSOLUTE_ENCODER_PORT);

    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(SHOULDER_kS, SHOULDER_kG, SHOULDER_kV, SHOULDER_kA);
    private final ProfiledPIDController shoulderFeedback = new ProfiledPIDController(SHOULDER_kP, SHOULDER_kI, SHOULDER_kD, SHOULDER_CONSTRAINTS);


    // -------- Second Joint ------ \\
    private final TalonFX secondJointMotor = new TalonFX(SECOND_MOTOR_ID);

    private final CANcoder secondJointEncoder = new CANcoder(SECOND_ABSOLUTE_ENCODER_PORT);

    private final ArmFeedforward wristFeedforward = new ArmFeedforward(WRIST_kS, WRIST_kG, WRIST_kV, WRIST_kA);
    private final ProfiledPIDController wristFeedback = new ProfiledPIDController(WRIST_kP, WRIST_kI, WRIST_kD, WRIST_CONSTRAINTS);


    public ArmSubsystem () {
        left_firstJointMotor.setInverted(LEFT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setInverted(RIGHT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setControl(new Follower(left_firstJointMotor.getDeviceID(), true));

        secondJointMotor.setInverted(SECOND_MOTOR_REVERSED);
    }

    @Override
    public void periodic() {
        // find target
        

        // lerp towards target
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

    public Translation2d getFirstLinkEndpoint() {
        final double angle1 = firstJointEncoder.getPosition().getValueAsDouble();

        return new Translation2d(sin(angle1), -cos(angle1)).times(L1);
    }



    public Translation2d getCurrentEndpoint() {
        final double angle1 = firstJointEncoder.getPosition().getValueAsDouble();
        final double angle2 = secondJointEncoder.getPosition().getValueAsDouble();

        final Translation2d a = new Translation2d(sin(angle1), -cos(angle1)).times(L1);
        final Translation2d b = new Translation2d(sin(angle1 + angle2), -cos(angle1 + angle2)).times(L2);

        return a.plus(b);
    }

    public static double secondJointIK(double x, double y) {
        return acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2));
    }

    public static double firstJointIK(double x, double y, double z2) {
        return atan2(y, x) - atan2(L2 * sin(z2), L1 + L2 * cos(z2));
    }

    public static double[] armInverseKinematics(double x, double y) {
        double z2 = secondJointIK(x, y);
        double z1 = firstJointIK(x, y, z2);
        return new double[] {z1, z2};
    }
}