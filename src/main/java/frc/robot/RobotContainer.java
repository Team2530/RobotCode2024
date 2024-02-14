// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();

    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
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
        driverXbox.x().onTrue(new InstantCommand(() -> {
            swerveDriveSubsystem.setHeadingRelative();
        }));
        operatorXbox.x().onTrue(new InstantCommand(() -> {
            armSubsystem.homeArm();
        }));
    }

    public void configureTestBindings() {
        driverXbox.a().whileTrue(armSubsystem.getWristQuasiCommand(SysIdRoutine.Direction.kForward));
        driverXbox.b().whileTrue(armSubsystem.getWristQuasiCommand(SysIdRoutine.Direction.kReverse));
        driverXbox.x().whileTrue(armSubsystem.getWristDynamicCommand(SysIdRoutine.Direction.kForward));
        driverXbox.y().whileTrue(armSubsystem.getWristDynamicCommand(SysIdRoutine.Direction.kReverse));
    
        driverXbox.button(13).whileTrue(armSubsystem.getShoulderQuasiCommand(SysIdRoutine.Direction.kForward));
        driverXbox.button(15).whileTrue(armSubsystem.getShoulderQuasiCommand(SysIdRoutine.Direction.kReverse));
        driverXbox.button(14).whileTrue(armSubsystem.getShoulderDynamicCommand(SysIdRoutine.Direction.kForward));
        driverXbox.button(12).whileTrue(armSubsystem.getShoulderDynamicCommand(SysIdRoutine.Direction.kReverse));
 
   }
   
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: Implement Auto Command
        return null;
    }
}
