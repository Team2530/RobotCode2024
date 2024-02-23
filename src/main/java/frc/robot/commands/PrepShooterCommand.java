package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

public class PrepShooterCommand extends Command {

    private final Intake intake;
    private final Shooter shooter;
    private final Arm arm;
    private double speed = 0;
    double starttime;

    public PrepShooterCommand(Intake intake, Shooter shooter, Arm arm) {
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
    }

    public PrepShooterCommand(Intake intake, Shooter shooter, double speed) {
        this.intake = intake;
        this.shooter = shooter;
        this.speed = speed;
        this.arm = null;
    }

    @Override
    public void initialize() {
        starttime = RobotController.getFPGATime();
        intake.coast();
        shooter.coast();

        if(arm != null) {
            shooter.setCustomPercent(arm.getPresetShooterSpeed());
        } else {
            shooter.setCustomPercent(speed);
        }
        
        SmartDashboard.putString("Shootake", "Spooling Shooter");
    }

    @Override
    public void execute() {
        // if ((RobotController.getFPGATime() - starttime) > 1.25) {
        //     cancel();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.setMode(ShooterMode.STOPPED);
        }
    }

    @Override
    public boolean isFinished() {
        // piece is behind front limit switch and is clear
        return shooter.isUpToSpeed();
        // if ((RobotController.getFPGATime() - starttime) > 10.0*10e6) {
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
