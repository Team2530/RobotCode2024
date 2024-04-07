package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class StageTwo extends ProfiledPIDSubsystem {

    private final TalonFX stageTwoMotor = new TalonFX(ArmConstants.STAGE_TWO_MOTOR_PORT);

    private final CANcoder stageTwoEncoder = new CANcoder(ArmConstants.STAGE_TWO_ENCODER_PORT);

    private double stageOneOffset = 0.0;

    public StageTwo() {
        super(ArmConstants.STAGE_TWO_PROFILEDPID);

        hardwareInit();

        setGoalDegrees(getMeasurement());
    }

    public void hardwareInit() {
        stageTwoMotor.setInverted(ArmConstants.STAGE_TWO_ISREVERSED);
        stageTwoMotor.setSafetyEnabled(false);
        stageTwoMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // setpoint doesn't really matter as we aren't using a feedforward as of right
        // now
        stageTwoMotor.set(output / 12d);

        SmartDashboard.putNumber("Two Output", output);
    }

    @Override
    protected double getMeasurement() {
        double actualPosition = (stageTwoEncoder.getPosition().getValueAsDouble()
                * (ArmConstants.STAGE_TWO_ENCODER_ISREVERSED ? -1 : 1));
        return Units.rotationsToRadians(ArmConstants.STAGE_TWO_ENCODER_OFFSET + actualPosition) + stageOneOffset;
    }

    public void updateStageOneOffset(double offset) {
        stageOneOffset = offset;
    }

    public void setGoalDegrees(double degrees) {
        setGoal(Units.degreesToRadians(degrees));
    }

    public double getGoal() {
        return m_controller.getGoal().position;
    }

}
