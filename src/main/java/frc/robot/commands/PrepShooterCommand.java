package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

public class PrepShooterCommand extends Command {

    private Intake intake;
    private Shooter shooter;
    double speed;
    double starttime;

    public PrepShooterCommand(Intake intake, Shooter shooter, double shootpercent) {
        this.intake = intake;
        this.shooter = shooter;
        speed = shootpercent;
    }

    @Override
    public void initialize() {
        starttime = RobotController.getFPGATime();

        
        intake.coast();
        shooter.coast();
        // shooter.setMode(ShooterMode.FULL);
        shooter.setCustomPercent(speed);
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
