package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class StageTwo extends ProfiledPIDSubsystem {

    private final TalonFX stageTwoMotor = new TalonFX(ArmConstants.STAGE_TWO_MOTOR_PORT);

    private final CANcoder stageTwoEncoder = new CANcoder(ArmConstants.STAGE_TWO_ENCODER_PORT);

    private double stageOneOffset = 0.0;

    private final DCMotor simStage2motor = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim jointsim = new SingleJointedArmSim(simStage2motor, ArmConstants.STAGE_TWO_RATIO,
            SingleJointedArmSim.estimateMOI(0.25, 0.1),
            Units.inchesToMeters(12), Units.degreesToRadians(-600), Units.degreesToRadians(600), true, 0.0);

    public StageTwo() {
        super(ArmConstants.STAGE_TWO_PROFILEDPID);

        stageTwoMotor.setInverted(ArmConstants.STAGE_TWO_ISREVERSED);
        stageTwoMotor.setSafetyEnabled(false);
        stageTwoMotor.setNeutralMode(NeutralModeValue.Brake);

        setGoalDegrees(getMeasurement());
    }

    @Override
    public void simulationPeriodic() {
        jointsim.setInput(stageTwoMotor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        jointsim.update(0.02);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        // m_encoderSim.setDistance(m_armSim.getAngleRads());
        SmartDashboard.putNumber("Arm sim angle", jointsim.getAngleRads());
        stageTwoEncoder
                .setPosition(
                        Units.radiansToRotations(jointsim.getAngleRads() - ArmConstants.STAGE_TWO_ENCODER_OFFSET));
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(jointsim.getCurrentDrawAmps()));

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

    public double getRawMeasurement() {
        double actualPosition = (stageTwoEncoder.getAbsolutePosition().getValueAsDouble()
                * (ArmConstants.STAGE_TWO_ENCODER_ISREVERSED ? -1 : 1));
        return Units.rotationsToRadians(ArmConstants.STAGE_TWO_ENCODER_OFFSET + actualPosition);
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
