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
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

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
    // private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();

    private final Targeting targeting = new Targeting(swerveDriveSubsystem);

    private final StageOne stageOne = new StageOne();
    private final StageTwo stageTwo = new StageTwo();
    private final Arm arm = new Arm(stageOne, stageTwo,targeting, operatorXbox.getHID());

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID(), targeting, arm);

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

        // Intake/intake shooter
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.INTAKE);
        }));

        // Low shoot preset
        operatorXbox.x().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.SHOOT_LOW);
        }));

        operatorXbox.povLeft().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.TRAP);
        }));

        // Amp preset
        operatorXbox.y().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.AMP);
        }));

        // High shoot preset
        operatorXbox.rightTrigger().and(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            // TODO Auto-generated method stub
            return operatorXbox.getRightTriggerAxis() > 0.1;
        } 
        }).onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.SHOOT_HIGH);
        }));
        
        // intake preset on climber start
        operatorXbox.povUp().onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.INTAKE);
        }));

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

        // Purge/ Spit command
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

        // source intake
        operatorXbox.button(7).onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            arm.setArmPreset(Presets.SOURCE);
                        })))
                .whileTrue(new SequentialCommandGroup(
                        new IntakeCommand(intake)).andThen(new ParallelCommandGroup(new InstantCommand(() -> {
                            arm.setArmPreset(Presets.STOW);
                        }),
                                new AlignNoteCommand(intake, shooter))));

        // Prepare shooting command
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
                                new PrepShooterCommand(intake, shooter, arm)
                        ))).onFalse(new SequentialCommandGroup(
                            new InstantCommand(() -> {
                            shooter.setMode(ShooterMode.STOPPED);
                        }),
                        new AlignNoteCommand(intake, shooter)));
                        ;

        // Recal climber               
        operatorXbox.button(8).onTrue(new InstantCommand(() -> {
            climber.leftArm.is_calibrated = false;            
            climber.rightArm.is_calibrated = false;
        }));

        // Driver trigger shoot
        driverXbox.leftTrigger().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return driverXbox.getLeftTriggerAxis() > 0.75 && shooter.isUpToSpeed();
            }
        }).onTrue(new ShootCommand(shooter, intake));

        driverXbox.a().onTrue(new InstantCommand(() -> {
            swerveDriveSubsystem.setRotationStyle(RotationStyle.Auto);
        })).onFalse(new InstantCommand(() -> {
            swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
        }));

        // // Fine tune on stage 2

        // // Fine tune stage 1
        // operatorXbox.leftStick().and(new BooleanSupplier() {
        //     @Override
        //     public boolean getAsBoolean() {
        //         return Math.abs(operatorXbox.getLeftY()) > 0.2;
        //     }
        // }).whileTrue(new InstantCommand(() -> {
        //     arm.setCustomGoal(arm.getStageOneDegrees(), arm.getStageTwoDegrees() + (operatorXbox.getLeftY() * ArmConstants.HUMAN_ARM_INPUT_P));
        // }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        NamedCommands.registerCommand("Shoot Close", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_LOW);}  
        )));
        NamedCommands.registerCommand("Shoot TM", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_TM);}
        )));
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
            new AlignNoteCommand(intake, shooter),
            new PrepNoteCommand(shooter, intake),
            new PrepShooterCommand(intake, shooter, 0.8),
            new ShootCommand(shooter, intake)
        ));
        NamedCommands.registerCommand("Spool", new SequentialCommandGroup(
            new AlignNoteCommand(intake, shooter),
            new PrepNoteCommand(shooter, intake),
            new PrepShooterCommand(intake, shooter, 0.8)
        ));
        NamedCommands.registerCommand("Intaking", new SequentialCommandGroup(
            new AutoIntakeCommand(intake, 3)
        ));
        NamedCommands.registerCommand("Pickup", 
            new InstantCommand(() -> {arm.setArmPreset(Presets.INTAKE);}));
        NamedCommands.registerCommand("Stow", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.STOW);}
        )));
         /* 
        NamedCommands.registerCommand("Shoot Close", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
            new WaitCommand(2),
            new AlignNoteCommand(intake, shooter),
            new PrepNoteCommand(shooter, intake),
            new PrepShooterCommand(intake, shooter, 0.8),
            new ShootCommand(shooter, intake)
        ));
        NamedCommands.registerCommand("Pickup", new SequentialCommandGroup(
            new InstantCommand(() -> {arm.setArmPreset(Presets.INTAKE);}),
            new IntakeCommand(intake)));
            */
        // Test Auto Week 0 
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
        //     new WaitCommand(2),
        //     new AlignNoteCommand(intake, shooter),
        //     new PrepNoteCommand(shooter, intake),
        //     new PrepShooterCommand(intake, shooter, 0.8),
        //     new InstantCommand(() -> System.out.println("HELLLLLOOO")),
        //     new ShootCommand(shooter, intake)
        // );
        return new PathPlannerAuto("4-top");

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
