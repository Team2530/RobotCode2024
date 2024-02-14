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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
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
    private final TalonFX shoulderMotor = new TalonFX(LEFT_SHOULDER_MOTOR_ID);
    private final TalonFX shoulderMotor_follower = new TalonFX(RIGHT_SHOULDER_MOTOR_ID);
    
    private final CANcoder shoulderEncoder = new CANcoder(SHOULDER_ABSOLUTE_ENCODER_PORT);

    private double shoulderSetpoint = 0;
    private double shoulderVelocity = 0;

    // -------- Second Joint ------ \\
    private final TalonFX wristMotor = new TalonFX(WRIST_MOTOR_ID);

    private final CANcoder wristEncoder = new CANcoder(WRIST_ABSOLUTE_ENCODER_PORT);

    private double wristSetpoint = 0;
    private double wristVelocity = 0;

    // -------- SysId -------------- \\
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private SysIdRoutine SHOULDER_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((voltage) -> {shoulderMotor.setVoltage(voltage.magnitude());}, this::sysIdShoulderMotorLog, this)
    );
    private SysIdRoutine WRIST_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((voltage) -> {wristMotor.setVoltage(voltage.magnitude());}, this::sysIdWristMotorLog, this)
    );

    // -------- Smartdashboard ----- \\
    Mechanism2d mech = new Mechanism2d(2, 2);
    MechanismRoot2d arm = mech.getRoot("arm", 0, 0);
    MechanismLigament2d shoulder = arm.append(new MechanismLigament2d("shoulder", 3, 90));
    MechanismLigament2d wrist = shoulder.append(new MechanismLigament2d("wrist", 1, 0));

    public static enum ControlState {
        Human,
        Homing
    }
    private ControlState controlState = ControlState.Human;
        
    // ---------------------------- \\


    public ArmSubsystem () {
        // persistant variables
        Preferences.initDouble(SHOULDER_OFFSET_KEY, SHOULDER_OFFSET_DEFAULT_RADIANS);
        Preferences.initDouble(WRIST_OFFSET_KEY, WRIST_OFFSET_DEFAULT_RADIANS);

        // motos n things        
        shoulderMotor.setInverted(SHOULDER_LEADER_REVERSED);
        shoulderMotor_follower.setControl(new Follower(shoulderMotor.getDeviceID(), SHOULDER_FOLLOWER_REVERSED));

        wristMotor.setInverted(WRIST_MOTOR_REVERSED);
    }

    @Override
    public void periodic() {
        shoulder.setAngle(shoulderEncoder.getPosition().getValue());
        wrist.setAngle(wristEncoder.getPosition().getValue());
        SmartDashboard.putData("Arm", mech); 

        switch (controlState) {
            case Human:
                // find target
        

                // lerp towards target
                // yippee.mp4
                break;
            case Homing:
                if (shoulderMotor.getReverseLimit().getValue().value == 1 || shoulderMotor_follower.getReverseLimit().getValue().value == 1) {
                    Preferences.setDouble(SHOULDER_OFFSET_KEY, shoulderEncoder.getPosition().getValue());
                    controlState = ControlState.Human;
                }
                break;
        }
        // shoulder
        shoulderMotor.setVoltage(
            SHOULDER_FEEDBACK.calculate(
                (shoulderEncoder.getPosition().getValueAsDouble() 
                    * (SHOULDER_ABSOLUTE_ENCODER_REVERSED ? -1 : 1)
                    ) + Preferences.getDouble(SHOULDER_OFFSET_KEY, SHOULDER_OFFSET_DEFAULT_RADIANS), 
                shoulderSetpoint
            ) + SHOULDER_FEEDFORWARD.calculate(shoulderSetpoint, shoulderVelocity)
        ); 
        // wrist
        wristMotor.setVoltage(
            WRIST_FEEDBACK.calculate(
                (wristEncoder.getPosition().getValueAsDouble() 
                    * (WRIST_ABSOLUTE_ENCODER_REVERSED ? -1 : 1)
                    ) + Preferences.getDouble(WRIST_OFFSET_KEY, WRIST_OFFSET_DEFAULT_RADIANS),
                wristSetpoint
            ) + WRIST_FEEDFORWARD.calculate(wristSetpoint, wristVelocity)
        );
    }



    public void setWrist(double setpoint) {
        if (controlState == ControlState.Human) {
            wristSetpoint = setpoint;
        }
    }
    public void setWristVelocity(double velocity) {
        if (controlState == ControlState.Human){
            wristVelocity = velocity;
        } 
    }

    public void setShoulder(double setpoint) {
        if (controlState == ControlState.Human){
            shoulderSetpoint = setpoint;
        } 
    }
    public void setShoulderVelocity(double velocity) {
        if (controlState == ControlState.Human){
            shoulderVelocity = velocity;
        }
    }

    public void homeArm() {
        controlState = ControlState.Homing;
        shoulderSetpoint = 0;
        wristSetpoint = 0;                       
    }

    

    public Translation2d getFirstLinkEndpoint() {
        final double angle1 = shoulderEncoder.getPosition().getValueAsDouble();

        return new Translation2d(sin(angle1), -cos(angle1)).times(L1);
    }

    public Translation2d getCurrentEndpoint() {
        final double angle1 = shoulderEncoder.getPosition().getValueAsDouble();
        final double angle2 = wristEncoder.getPosition().getValueAsDouble();

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



    public void sysIdShoulderMotorLog(SysIdRoutineLog log){
         log.motor("shoulder-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shoulderMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(shoulderEncoder.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(shoulderMotor.getVelocity().getValue(), MetersPerSecond));
        log.motor("shoulder-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shoulderMotor_follower.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(shoulderMotor_follower.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(shoulderMotor_follower.getVelocity().getValue(), MetersPerSecond));
    }

    public void sysIdWristMotorLog(SysIdRoutineLog log){
         log.motor("wrist")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            wristMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(wristMotor.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(wristMotor.getVelocity().getValue(), MetersPerSecond));
    }

    public Command getShoulderQuasiCommand(Direction direction){
        return SHOULDER_sysIdRoutine.quasistatic(direction);
    }

    public Command getShoulderDynamicCommand(Direction direction){
        return SHOULDER_sysIdRoutine.dynamic(direction);
    }

    public Command getWristQuasiCommand(Direction direction){
        return WRIST_sysIdRoutine.quasistatic(direction);
    }
    
    public Command getWristDynamicCommand(Direction direction){
        return WRIST_sysIdRoutine.dynamic(direction);
    }
}