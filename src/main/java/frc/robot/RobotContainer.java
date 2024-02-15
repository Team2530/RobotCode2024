// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
<<<<<<< HEAD
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;
=======
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Presets;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

import java.util.function.BooleanSupplier;
>>>>>>> intake-shooter

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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

    // TODO: Uncomment when ready to test DRIVE!
    // private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    // ----------- Commands ---------- \\

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        // swerveDriveSubsystem.setDefaultCommand(normalDrive);
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
<<<<<<< HEAD
=======
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.INTAKE);
        }));

        operatorXbox.b().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.STOW);
        }));
>>>>>>> intake-shooter
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("AMP");
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveDriveSubsystem;
    }
}
