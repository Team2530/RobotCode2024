package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.ClimberArm;

public class ClimberSubsystem extends SubsystemBase {
    public ClimberArm leftArm = new ClimberArm(ClimberConstants.LEFT_CLIMBER_CANID,
            ClimberConstants.LEFT_CLIMBER_INVERTED);
    public ClimberArm rightArm = new ClimberArm(ClimberConstants.RIGHT_CLIMBER_CANID,
            ClimberConstants.RIGHT_CLIMBER_INVERTED);
    public AHRS navX;

    public ClimberSubsystem(AHRS navX) {
        this.navX = navX;
        hardwareInit();
    }

    public void hardwareInit() {
        leftArm.hardwareInit();
        rightArm.hardwareInit();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Climber Position", new double[] {
                leftArm.motor.getEncoder().getPosition(),
                rightArm.motor.getEncoder().getPosition()
        });

        leftArm.periodic();
        rightArm.periodic();
    }

    public void climb(double throtle) {
        climb_tilt(throtle, 0.0);
    }

    public void deploy() {
        leftArm.set(1);
        rightArm.set(1);
    }

    public void idle() {
        leftArm.set(0.0);
        rightArm.set(0.0);
    }

    /// Tilt positive rolls clockwise
    public void climb_tilt(double climb, double tilt) {
        // if (leftArm.isRetracted() && rightArm.isRetracted()) {
        // stop();
        // return;
        // }
        leftArm.set(climb + tilt);
        rightArm.set(climb - tilt);
    }

    public void stop() {
        leftArm.stop();
        rightArm.stop();
    }
}
