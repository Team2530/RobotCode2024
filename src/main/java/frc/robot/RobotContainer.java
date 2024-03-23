// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Presets;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.LEDstripOne;

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

    private final CommandXboxController debugXbox = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();

    private final Targeting targeting = new Targeting(swerveDriveSubsystem);

    private final StageOne stageOne = new StageOne();
    private final StageTwo stageTwo = new StageTwo();

    private final Intake intake = new Intake(driverXbox, operatorXbox);
    private final Shooter shooter = new Shooter();
    private final Arm arm = new Arm(stageOne, stageTwo, shooter, targeting, operatorXbox.getHID());

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID(), targeting,
            arm);

    private final LEDstripOne m_stripOne = new LEDstripOne(9, intake, shooter, arm, swerveDriveSubsystem, normalDrive);

    // ----------- Commands ---------- \\

    private final ClimberSubsystem climber = new ClimberSubsystem(swerveDriveSubsystem.navX);
    private final ClimberCommand climberCommand = new ClimberCommand(climber, operatorXbox.getHID());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        NamedCommands.registerCommand("Shoot Close", new SequentialCommandGroup(
                new InstantCommand(() -> {
                    arm.setArmPreset(Presets.SHOOT_LOW);
                })));
        NamedCommands.registerCommand("Shoot TM", new SequentialCommandGroup(
                new InstantCommand(() -> {
                    arm.setArmPreset(Presets.SHOOT_TM);
                })));
        NamedCommands.registerCommand("Shoot AMP", new SequentialCommandGroup(
                new InstantCommand(() -> {
                    arm.setArmPreset(Presets.AUTO_AMP);
                })));
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                new WaitUntilCommand(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return shooter.isUpToSpeed();
                    }
                }),
                new ShootCommand(shooter, intake)));
        NamedCommands.registerCommand("Spool", new SequentialCommandGroup(
                new AlignNoteCommand(intake, shooter),
                new PrepNoteCommand(intake),
                new PrepShooterCommand(shooter, 0.8)));
        NamedCommands.registerCommand("SpoolAMP", new SequentialCommandGroup(
                new AlignNoteCommand(intake, shooter),
                new PrepNoteCommand(intake),
                new PrepShooterCommand(shooter, 0.5)));
        NamedCommands.registerCommand("Intaking", new SequentialCommandGroup(
                new AutoIntakeCommand(intake, 3)));
        NamedCommands.registerCommand("Intaking 5", new SequentialCommandGroup(
                new AutoIntakeCommand(intake, 10)));
        NamedCommands.registerCommand("Pickup",
                new InstantCommand(() -> {
                    arm.setArmPreset(Presets.INTAKE);
                }));
        NamedCommands.registerCommand("Stow", new SequentialCommandGroup(
                new InstantCommand(() -> {
                    arm.setArmPreset(Presets.STOW);
                })));
        /*
         * NamedCommands.registerCommand("Shoot Close", new SequentialCommandGroup(
         * new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
         * new WaitCommand(2),
         * new AlignNoteCommand(intake, shooter),
         * new PrepNoteCommand(shooter, intake),
         * new PrepShooterCommand(intake, shooter, 0.8),
         * new ShootCommand(shooter, intake)
         * ));
         * NamedCommands.registerCommand("Pickup", new SequentialCommandGroup(
         * new InstantCommand(() -> {arm.setArmPreset(Presets.INTAKE);}),
         * new IntakeCommand(intake)));
         */
        // Test Auto Week 0
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
        // new WaitCommand(2),
        // new AlignNoteCommand(intake, shooter),
        // new PrepNoteCommand(shooter, intake),
        // new PrepShooterCommand(intake, shooter, 0.8),
        // new InstantCommand(() -> System.out.println("HELLLLLOOO")),
        // new ShootCommand(shooter, intake)
        // );

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.onChange(new Consumer<Command>() {
            public void accept(Command t) {
                m_stripOne.updateAutoStartPosition(autoChooser.getSelected().getName());
            };
        });
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
        climber.setDefaultCommand(climberCommand);
    }

    // Command shootAction = 
    // Command alignAction = ; // Self-deadlines
    // Command spoolAction = 
    // Command intakeAction = ;

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
        debugXbox.a().onTrue(new AlignNoteCommand(intake, shooter));
        debugXbox.b().whileTrue(new AlignNoteCommand(intake, shooter).andThen(new PrepNoteCommand(intake)));
        debugXbox.x().whileTrue(new IntakeCommand(intake).andThen(new AlignNoteCommand(intake, shooter))).onFalse(new InstantCommand(() -> {
            intake.setMode(IntakeMode.STOPPED);
        }));

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
            arm.setArmPreset(Presets.AUTO_AMP);
        }));

        operatorXbox.button(10).onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.SHOOT_MANUAL);
        }));

        operatorXbox.button(9).onTrue(new InstantCommand(() -> {
            arm.setArmPreset(Presets.SHOOT_SHUTTLE);
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

        /*
         * // intake preset on climber start
         * operatorXbox.povUp().onTrue(new InstantCommand(() -> {
         * arm.setArmPreset(Presets.CLIMB);
         * }));
         */

        // set arm to intake, once has happened, retract the arm and center the note
        // operatorXbox.leftBumper().onTrue(
        //         new SequentialCommandGroup(
        //                 new InstantCommand(() -> {
        //                     arm.setArmPreset(Presets.INTAKE);
        //                 })))
        //         .whileTrue(new SequentialCommandGroup(
        //                 new IntakeCommand(intake)).andThen(new ParallelCommandGroup(new InstantCommand(() -> {
        //                     arm.setArmPreset(Presets.STOW);
        //                 }),new AlignNoteCommand(intake, shooter))));

        operatorXbox.leftBumper().onTrue(new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> {
                        arm.setArmPreset(Presets.STOW);
                    }),
                    new InstantCommand(() -> {
                        arm.setArmPreset(Presets.INTAKE);
                    }),
                intake.containsNote())))
                .whileTrue(new IntakeCommand(intake).andThen(new AlignNoteCommand(intake, shooter)).andThen(new InstantCommand(() -> {
                            arm.setArmPreset(Presets.STOW);
                        }))).onFalse(new InstantCommand(() -> {
                            intake.setMode(IntakeMode.STOPPED);
                        }));

        // Purge/spit command
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
                new ConditionalCommand(
                    new InstantCommand(() -> {
                        arm.setArmPreset(Presets.STOW);
                    }),
                    new InstantCommand(() -> {
                        arm.setArmPreset(Presets.SOURCE);
                    }),
                intake.containsNote())))
                .whileTrue(new IntakeCommand(intake).andThen(new AlignNoteCommand(intake, shooter)).andThen(new InstantCommand(() -> {
                            arm.setArmPreset(Presets.STOW);
                        }))).onFalse(new InstantCommand(() -> {
                            intake.setMode(IntakeMode.STOPPED);
                        }));

        // Prepare shooting command
        operatorXbox.rightBumper().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                // note in shootake
                return intake.getIntakeSideLimitClosed() || intake.getShooterSideLimitClosed();
            }
        }).onTrue(new SequentialCommandGroup(
            new AlignNoteCommand(intake, shooter),
            new ParallelCommandGroup(
                new PrepNoteCommand(intake),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new PrepShooterCommand(shooter, arm)
                )
            )
        ).raceWith(new WaitUntilCommand(operatorXbox.rightBumper().negate()))
        ).onFalse(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            shooter.setMode(ShooterMode.STOPPED);
                        }),
                        new WaitUntilCommand(new BooleanSupplier() {
                            @Override
                            public boolean getAsBoolean() {
                                return shooter.isStopped();
                            }
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
                return driverXbox.getLeftTriggerAxis() > 0.25 && shooter.isUpToSpeed();
            }
        }).onTrue(new ShootCommand(shooter, intake).deadlineWith(new WaitCommand(1.0)));

        driverXbox.a().onTrue(new InstantCommand(() -> {
            swerveDriveSubsystem.setRotationStyle(RotationStyle.Auto);
        })).onFalse(new InstantCommand(() -> {
            swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
        }));

        // SOFT RESET
        driverXbox.button(7).onTrue(new InstantCommand(() -> {
            arm.hardwareInit();
            shooter.hardwareInit();
            intake.hardwareInit();
            climber.hardwareInit();
        }));

        // // Fine tune on stage 2

        // // Fine tune stage 1
        // operatorXbox.leftStick().and(new BooleanSupplier() {
        // @Override
        // public boolean getAsBoolean() {
        // return Math.abs(operatorXbox.getLeftY()) > 0.2;
        // }
        // }).whileTrue(new InstantCommand(() -> {
        // arm.setCustomGoal(arm.getStageOneDegrees(), arm.getStageTwoDegrees() +
        // (operatorXbox.getLeftY() * ArmConstants.HUMAN_ARM_INPUT_P));
        // }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

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
