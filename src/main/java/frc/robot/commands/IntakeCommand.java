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
        intake.setIntakeMode(IntakeMode.INTAKING);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMode(IntakeMode.STOPED);
    }

    @Override
    public boolean isFinished() {
        return intake.getFrontLimitClosed();
    }
    
}
