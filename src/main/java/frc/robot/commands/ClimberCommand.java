package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.ClimberArm.DeployMode;

public class ClimberCommand extends Command {
    private final XboxController xbox;

    private ClimberSubsystem climber;

    int pastpov;

    public ClimberCommand(ClimberSubsystem climber, XboxController xbox) {
        this.climber = climber;
        this.xbox = xbox;
        addRequirements(climber);
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        // boolean climbing = (xbox.getLeftBumper() || xbox.getRightBumper()) &&
        // (!xbox.getAButton());

        // boolean manual = false;
        // if (Math.abs(xbox.getLeftY()) > 0.025) {
        // climber.rightArm.set(-MathUtil.applyDeadband(xbox.getLeftY(), 0.025));
        // manual = true;
        // }
        // if (Math.abs(xbox.getRightY()) > 0.025) {
        // climber.leftArm.set(-MathUtil.applyDeadband(xbox.getRightY(), 0.025));
        // manual = true;
        // }

        // if (manual == false) {
        switch (xbox.getPOV()) {
            case 180:
                climber.deploy();
                break;

            case 0:
                // double tilt_angle = climber.navX.getRoll();
                // double roll_compensation = (-tilt_angle / 90.0f) * ClimberConstants.ROLL_kP;
                // climber.climb_tilt(-1.0, roll_compensation);
                climber.climb(-1.0);
                break;
            case 270:
                if (pastpov != 270)
                    climber.rightArm
                            .setDeployMode(climber.rightArm.getDeployMode() == DeployMode.Extend ? DeployMode.FlipUp
                                    : DeployMode.Extend);
                break;
            case 90:
                if (pastpov != 90)
                    climber.leftArm
                            .setDeployMode(climber.leftArm.getDeployMode() == DeployMode.Extend ? DeployMode.FlipUp
                                    : DeployMode.Extend);
                break;
            default:
                climber.idle();
                break;
        }

        pastpov = xbox.getPOV();
        // }

        // climber.leftArm.setDeployMode(DeployMode.Extend);
        // climber.rightArm.setDeployMode(DeployMode.Extend);

        SmartDashboard.putBoolean("Climber L Flipup", climber.leftArm.getDeployMode() == DeployMode.FlipUp);
        SmartDashboard.putBoolean("Climber R Flipup", climber.rightArm.getDeployMode() == DeployMode.FlipUp);

        // MANUAL OVERRIDE
        // if (xbox.getAButton()) {
        // climber.leftArm.motor.set(xbox.getLeftY());
        // climber.rightArm.motor.set(xbox.getRightY());
        // }

        // if (command == ClimberState.Stow) {
        // climber.stow();
        // } else if (command == ClimberState.Stow) {

        // }

        // else if (climbing) {
        // double tilt_angle = climber.navX.getRoll();
        // double roll_compensation = (-tilt_angle / 90.0f) * ClimberConstants.ROLL_kP;

        // climber.climb_tilt(c_throttle, roll_compensation);
        // } else {
        // climber.stop();
        // }
    }

    // HACK: Only leave this initialize() method if the climber needs to calibrate
    // every time the bot enables...
    // @Override
    // public void initialize() {
    // climber.leftArm.is_calibrated = false;
    // climber.rightArm.is_calibrated = false;
    // }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}