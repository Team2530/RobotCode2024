package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.brake();
        intake.setForwardLimitEnabled(true);
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
        return intake.getFrontLimitClosed();
    }
    
}
