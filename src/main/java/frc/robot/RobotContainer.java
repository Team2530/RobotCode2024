// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;
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
    //private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();

    private final StageOne stageOne = new StageOne();
    private final StageTwo stageTwo = new StageTwo();
    private final Arm arm = new Arm(stageOne, stageTwo);

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final Intake intake = new Intake(driverXbox, operatorXbox);
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

        operatorXbox.leftTrigger().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                return operatorXbox.getLeftTriggerAxis() > 0.1;
            }
        }).onTrue(new InstantCommand(() -> {
            intake.setCustomPercent(-operatorXbox.getLeftTriggerAxis());
            shooter.setCustomPercent(operatorXbox.getLeftTriggerAxis());
        })).onFalse(new InstantCommand(() -> {
            intake.setCustomPercent(0.0);
            shooter.setCustomPercent(0.0);
        }));

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
                                new PrepShooterCommand(intake, shooter, 0.8)
                        // new InstantCommand(() -> {
                        // shooter.coast();
                        // shooter.setMode(ShooterMode.STOPPED);
                        // })
                        ))).onFalse(new SequentialCommandGroup(
                            new InstantCommand(() -> {
                            shooter.setMode(ShooterMode.STOPPED);
                        }),
                        new AlignNoteCommand(intake, shooter)));

        operatorXbox.button(8).onTrue(new InstantCommand(() -> {
            climber.leftArm.is_calibrated = false;            
            climber.rightArm.is_calibrated = false;
        }));

        driverXbox.leftTrigger().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return driverXbox.getLeftTriggerAxis() > 0.75 && shooter.isUpToSpeed();
            }
        }).onTrue(new ShootCommand(shooter, intake));

        //Vision version v1 button bindings
        // // Left tag
        // driverXbox.a().whileTrue(new GoToAprilTagCommand(swerveDriveSubsystem, limeLightSubsystem, AprilTagPosition.LEFT, null));
        // // Right tag
        // driverXbox.b().whileTrue(new GoToAprilTagCommand(swerveDriveSubsystem, limeLightSubsystem, AprilTagPosition.RIGHT, null));
        // // single/center tag
        // driverXbox.y().whileTrue(new GoToAprilTagCommand(swerveDriveSubsystem, limeLightSubsystem, AprilTagPosition.CENTER, null));
        // // Search and go to AMP April Tag
        // driverXbox.povRight().whileTrue(new GoToAprilTagCommand(swerveDriveSubsystem, limeLightSubsystem, null, AprilTagType.AMP));
        // // Search and got to AMP April Tag
        // driverXbox.povDown().whileTrue(new GoToAprilTagCommand(swerveDriveSubsystem, limeLightSubsystem, null, AprilTagType.SPEAKER));
        // // Search and got to AMP April Tag
        // driverXbox.povUp().whileTrue(new GoToAprilTagCommand(swerveDriveSubsystem, limeLightSubsystem, null, AprilTagType.STAGE));

        //Vision version v2 button bindings
        driverXbox.y().whileTrue(new GoToAprilTagCommandUsingPoseEstimator(swerveDriveSubsystem, swerveDriveSubsystem::getPose, AprilTagPosition.LEFT, AprilTagType.SPEAKER));
        //driverXbox.a().whileTrue(new GoToAprilTagCommandUsingPoseEstimator(swerveDriveSubsystem, swerveDriveSubsystem::getPose, AprilTagPosition.RIGHT, AprilTagType.SOURCE));
        //driverXbox.b().whileTrue(new GoToAprilTagCommandUsingPoseEstimator(swerveDriveSubsystem, swerveDriveSubsystem::getPose, AprilTagPosition.CENTER, AprilTagType.AMP));
        driverXbox.povRight().whileTrue(new GoToAprilTagCommandUsingPoseEstimator(swerveDriveSubsystem, swerveDriveSubsystem::getPose, AprilTagPosition.RIGHT, AprilTagType.STAGE));
        //driverXbox.povUp().whileTrue(new GoToAprilTagCommandUsingPoseEstimator(swerveDriveSubsystem, swerveDriveSubsystem::getPose, AprilTagPosition.CENTER, AprilTagType.STAGE));
        //driverXbox.povLeft().whileTrue(new GoToAprilTagCommandUsingPoseEstimator(swerveDriveSubsystem, swerveDriveSubsystem::getPose, AprilTagPosition.LEFT, AprilTagType.STAGE));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      /**  NamedCommands.registerCommand("Shoot Close", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
            new WaitCommand(2),
            new AlignNoteCommand(intake, shooter),
            new PrepNoteCommand(shooter, intake),
            new PrepShooterCommand(intake, shooter, 0.8),
            new ShootCommand(shooter, intake)
        )); */
       /** NamedCommands.registerCommand("Pickup", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.INTAKE);}),
            new IntakeCommand(intake)));*/
   //     return new PathPlannerAuto("Test Auto");
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
        //     new WaitCommand(2),
        //     new AlignNoteCommand(intake, shooter),
        //     new PrepNoteCommand(shooter, intake),
        //     new PrepShooterCommand(intake, shooter, 0.8),
        //     new InstantCommand(() -> System.out.println("HELLLLLOOO")),
        //     new ShootCommand(shooter, intake)
        // );

        return new PathPlannerAuto("AMP");
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
