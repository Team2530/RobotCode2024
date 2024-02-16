// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Presets;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();

    private final StageOne stageOne = new StageOne();
    private final StageTwo stageTwo = new StageTwo();
    private final Arm arm = new Arm(stageOne, stageTwo);

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final Intake intake = new Intake(driverXbox);
    private final Shooter shooter = new Shooter();

    // ----------- Commands ---------- \\

    private final ClimberSubsystem climber = new ClimberSubsystem(swerveDriveSubsystem.navX);
    private final ClimberCommand climberCommand = new ClimberCommand(climber, operatorXbox.getHID());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
        climber.setDefaultCommand(climberCommand);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Stow intake/shooter
        operatorXbox.b().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.STOW);
        }));

        // INtake/intake shooter
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.INTAKE);
        }));

        operatorXbox.x().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.SHOOT_HIGH);
        }));

        operatorXbox.y().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.AMP);
        }));

        // ? old intake
        // operatorXbox.y().and(new BooleanSupplier() {
        // public boolean getAsBoolean() {
        // return !intake.getFrontLimitClosed();
        // }
        // }).onTrue(
        // new IntakeCommand(intake).raceWith(new
        // WaitUntilCommand(operatorXbox.y().negate())));

        // set arm to intake, once has happened, retract the arm and center the note
        operatorXbox.leftBumper().onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.setArmPreset(Presets.INTAKE);
                        })))
                .whileTrue(new SequentialCommandGroup(
                        new IntakeCommand(intake)).andThen(new ParallelCommandGroup(new InstantCommand(() -> {
                            arm.setArmPreset(Presets.STOW);
                        }),
                                new AlignNoteCommand(intake, shooter))));

        // shoot command
        operatorXbox.rightBumper().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                // note in shootake
                return intake.getReverseLimitClosed() || intake.getFrontLimitClosed();
            }
        }).onTrue(
                new ParallelRaceGroup(
                        new WaitUntilCommand(operatorXbox.rightBumper().negate()),
                        new SequentialCommandGroup(
                                new AlignNoteCommand(intake, shooter),
                                new PrepNoteCommand(shooter, intake),
                                new PrepShooterCommand(intake, shooter, 0.4),
                                new ShootCommand(shooter, intake)
                        // new InstantCommand(() -> {
                        // shooter.coast();
                        // shooter.setMode(ShooterMode.STOPPED);
                        // })
                        )));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        NamedCommands.registerCommand("Shoot Close", Commands.print("Implement actual comand here to make it shoot"));
        NamedCommands.registerCommand("Shoot Far", Commands.print("Shoot from farther away look at (4-top)"));
        NamedCommands.registerCommand("Pickup", Commands.print("Have continuous until picked up???"));
        
        return new PathPlannerAuto("4-close");
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveDriveSubsystem;
    }

    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }

    public CommandXboxController getOperatorXbox() {
        return operatorXbox;
    }
}
