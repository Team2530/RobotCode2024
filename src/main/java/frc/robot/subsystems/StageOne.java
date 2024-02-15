package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class StageOne extends ProfiledPIDSubsystem {

    // Motors
    private final TalonFX stageOneLeader = new TalonFX(ArmConstants.STAGE_ONE_MOTOR_L);
    private final TalonFX stageOneFollower = new TalonFX(ArmConstants.STAGE_ONE_MOTOR_R);

    // Cancoder
    private final CANcoder stageOneEncoder = new CANcoder(ArmConstants.STAGE_ONE_ENCODER_PORT);

    private final ArmFeedforward feedForward = ArmConstants.STAGE_ONE_FEEDFORWARD;

    public StageOne() {
        super(ArmConstants.STAGE_ONE_PROFILEDPID);

        stageOneLeader.setInverted(ArmConstants.L_STAGE_ONE_ISREVERSED);
        stageOneFollower
                .setControl(new Follower(stageOneLeader.getDeviceID(), ArmConstants.FOLLOWER_STAGE_ONE_ISREVERSED));

        stageOneLeader.setSafetyEnabled(false);
        stageOneFollower.setSafetyEnabled(false);

        stageOneLeader.setNeutralMode(NeutralModeValue.Brake);        
        stageOneFollower.setNeutralMode(NeutralModeValue.Brake);


        setGoalDegrees(getMeasurement());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Left FWD Limit", stageOneLeader.getReverseLimit().getValueAsDouble() == 1);
        SmartDashboard.putBoolean("Right FWD Limit", stageOneFollower.getReverseLimit().getValueAsDouble() == 1);
        SmartDashboard.putBoolean("Right BWD Limit", stageOneLeader.getForwardLimit().getValueAsDouble() == 1);
        super.periodic();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // calculated feedworward
        double feed = feedForward.calculate(setpoint.position, setpoint.velocity);
        // apply feedforward to PID
        stageOneLeader.set((output + feed) / 12d);
    }

    @Override
    protected double getMeasurement() {
        double actualPosition = (stageOneEncoder.getAbsolutePosition().getValueAsDouble() * (ArmConstants.STAGE_ONE_ENCODER_ISREVERSED ? -1 : 1));
        return Units.rotationsToRadians(ArmConstants.STAGE_ONE_ENCODER_OFFSET + actualPosition);
    }

    public void setGoalDegrees(double degrees) {
        setGoal(Units.degreesToRadians(degrees));
        enable();
    }

}
