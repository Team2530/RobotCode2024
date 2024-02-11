package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

    // -------- SysId -------------- \\
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private SysIdRoutine SHOULDER_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((voltage) -> {left_firstJointMotor.setVoltage(voltage.magnitude());}, this::sysIdShoulderMotorLog, this)
    );
    private SysIdRoutine WRIST_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((voltage) -> {secondJointMotor.setVoltage(voltage.magnitude());}, this::sysIdWristMotorLog, this)
    );

    // -------- Smartdashboard ----- \\
    Mechanism2d mech = new Mechanism2d(2, 2);
    MechanismRoot2d arm = mech.getRoot("arm", 0, 0);
    MechanismLigament2d shoulder = arm.append(new MechanismLigament2d("shoulder", 3, 90));
    MechanismLigament2d wrist = shoulder.append(new MechanismLigament2d("wrist", 1, 0));

    public ArmSubsystem () {
        left_firstJointMotor.setInverted(LEFT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setInverted(RIGHT_FIRST_MOTOR_REVERSED);
        right_firstJointMotor.setControl(new Follower(left_firstJointMotor.getDeviceID(), true));

        secondJointMotor.setInverted(SECOND_MOTOR_REVERSED);
    }

    @Override
    public void periodic() {
        shoulder.setAngle(firstJointEncoder.getPosition().getValue());
        wrist.setAngle(secondJointEncoder.getPosition().getValue());
        SmartDashboard.putData("Arm", mech); 
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

    public void sysIdShoulderMotorLog(SysIdRoutineLog log){
         log.motor("shoulder-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            left_firstJointMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(firstJointEncoder.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(left_firstJointMotor.getVelocity().getValue(), MetersPerSecond));
        log.motor("shoulder-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            right_firstJointMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(right_firstJointMotor.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(right_firstJointMotor.getVelocity().getValue(), MetersPerSecond));
    }

    public void sysIdWristMotorLog(SysIdRoutineLog log){
         log.motor("wrist")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            secondJointMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(secondJointMotor.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(secondJointMotor.getVelocity().getValue(), MetersPerSecond));
    }

    public Command shoulderQuasiCommand(Direction direction){
        return SHOULDER_sysIdRoutine.quasistatic(direction);
    }

    public Command shoulderDynamicCommand(Direction direction){
        return SHOULDER_sysIdRoutine.dynamic(direction);
    }

    public Command wristQuasiCommand(Direction direction){
        return WRIST_sysIdRoutine.quasistatic(direction);
    }
    
    public Command wristDynamicCommand(Direction direction){
        return WRIST_sysIdRoutine.dynamic(direction);
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