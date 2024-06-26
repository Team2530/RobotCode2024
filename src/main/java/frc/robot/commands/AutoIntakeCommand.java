package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class AutoIntakeCommand extends Command {
    private final Intake intake;
    private double ITime = 0.0;
    private double time = 0;

    public AutoIntakeCommand(Intake intake, double time) {
        this.intake = intake;
        this.time = time;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        ITime = Timer.getFPGATimestamp();
        intake.brake();
        intake.setShooterLimitEnabled(true);
        intake.setMode(IntakeMode.INTAKING);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeMode.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return intake.getShooterSideLimitClosed() || ((Timer.getFPGATimestamp() - ITime) > time) ;
    }
    
}
