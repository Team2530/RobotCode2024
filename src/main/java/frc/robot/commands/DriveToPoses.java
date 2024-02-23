package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.Presets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPoses extends Command {

    private static final double TRANSLATION_TOLERANCE = 0.3;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(10.0);

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      .5,
      1);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      0.4,
      1);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final SwerveSubsystem SwerveSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final Optional<Alliance> ally = DriverStation.getAlliance();
  private final Arm arm;

  public DriveToPoses(
        SwerveSubsystem SwerveSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose, Arm arm) {
    this(SwerveSubsystem, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, arm);
  }

  public DriveToPoses(
        SwerveSubsystem SwerveSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints, Arm arm) {
    this.SwerveSubsystem = SwerveSubsystem;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.arm = arm;

    xController = new ProfiledPIDController(1, 0, .02, xyConstraints);
    yController = new ProfiledPIDController(1.5, 0, .02, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(.9, 0, .08, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(SwerveSubsystem);
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

    @Override
    public void initialize() {
        new InstantCommand(() -> {
            arm.setArmPreset(Presets.STOW);
        });
        resetPIDControllers();
        var pose = goalPose;
        if (ally.get() == Alliance.Red) {
            Translation2d transformedTranslation = new Translation2d(pose.getX(), 8.1026 - pose.getY());
            Rotation2d transformedHeading = pose.getRotation().times(-1);
            pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());  
    }

    @Override
  public void execute() {
    var robotPose = poseProvider.get();
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    SwerveSubsystem.setChassisSpeedsAUTO(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

    @Override
    public boolean isFinished() {
      return atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.stopDrive();
  }
}
