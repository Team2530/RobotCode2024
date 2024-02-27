package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;

public class Arm extends SubsystemBase {

  public enum Presets {
    // Stage one angle is 0 refrenced from the horizontal
    // stage 2 angle is refrenced as zero relative to stage one, intake pointing
    // out the front when the arm is vertical, and the intake horizontal
    STOW(-0.37, 181),
    SHOOT_LOW(19, 48),
    SHOOT_TM(19, 38),
    INTAKE(-14.7, 37.2),
    AMP(101, 125),
    SHOOT_HIGH(90, 40),
    STARTING_CONFIG(0, 90),
    SOURCE(42, 132),
    CUSTOM(0, 0);

    private double s1angle;
    private double s2angle;

    private Presets(double s1angle, double s2angle) {
      this.s1angle = s1angle;
      this.s2angle = s2angle;
    }

  }

  private final StageOne stageOne;
  private final StageTwo stageTwo;
  private final XboxController operatorXbox;
  private final SwerveSubsystem swerveSubsystem;

  private Presets currentPreset = Presets.STARTING_CONFIG;

  public Arm(StageOne stageOne, StageTwo stageTwo, SwerveSubsystem swerveSubsystem, XboxController operatorXbox) {
    this.stageOne = stageOne;
    this.stageTwo = stageTwo;
    this.operatorXbox = operatorXbox;
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    stageTwo.updateStageOneOffset(stageOne.getMeasurement());

    SmartDashboard.putNumber("Stage One Encoder", Units.radiansToDegrees(stageOne.getMeasurement()));
    SmartDashboard.putNumber("Stage Two Encoder", Units.radiansToDegrees(stageTwo.getMeasurement()));

    SmartDashboard.putNumber("Shoot optimal angle", getAngleToTarget(ArmConstants.SHOOTER_LOW_X_OFFSET, ArmConstants.SHOOTER_LOW_HEIGHT));

    if(currentPreset == Presets.SHOOT_LOW) {
      stageTwo.setGoalDegrees(getAngleToTarget(ArmConstants.SHOOTER_LOW_X_OFFSET, ArmConstants.SHOOTER_LOW_HEIGHT));
    } else if(currentPreset == Presets.SHOOT_HIGH) {
      stageTwo.setGoalDegrees(getAngleToTarget(ArmConstants.SHOOTER_HIGH_X_OFFSET, ArmConstants.SHOOTER_HIGH_HEIGHT));
    }
  }

  public void setArmPreset(Presets preset) {
    // only change if new goal
    if (currentPreset != preset) {
      stageOne.setGoalDegrees(preset.s1angle);
      stageTwo.setGoalDegrees(preset.s2angle);
      currentPreset = preset;
      SmartDashboard.putString("Arm Preset", "Moving to " + currentPreset.name());

      if (DriverStation.isEnabled()) {
        stageOne.enable();
        stageTwo.enable();
      }
    }
  }

  /**
   * Sets the arm to a custom goal for stage 1 and stage 2
   * @param stageOneDegrees (shoulder) degrees
   * @param stageTwoDegrees (wrist) degrees
   */
  public void setCustomGoal(double stageOneDegrees, double stageTwoDegrees) {
    stageOneDegrees = MathUtil.clamp(stageOneDegrees, Presets.INTAKE.s1angle, Presets.AMP.s1angle);
    stageTwoDegrees = MathUtil.clamp(stageTwoDegrees, Presets.INTAKE.s2angle, Presets.STOW.s2angle);
    stageOne.setGoalDegrees(stageOneDegrees);
    stageTwo.setGoalDegrees(stageTwoDegrees);

  }

  public double getPresetShooterSpeed() {
    switch (currentPreset) {
      case SHOOT_HIGH:
        return 0.8;
      case SHOOT_LOW:
        return 1;
      case AMP:
        return 0.5;
      default:
        return 0.8;
    }
  }

  public double getStageOneDegrees() {
    return stageOne.getMeasurement();
  }

  public double getStageTwoDegrees() {
    return stageTwo.getMeasurement();
  }

  private double getDistanceToTarget() {
    return swerveSubsystem.getPose().getTranslation().getDistance(FieldConstants.getSpeakerPosition());
  }

  private double getAngleToTarget(double shooterHorizontal, double shooterVertical) {
    double x = getDistanceToTarget() + shooterHorizontal;
    double y = FieldConstants.SPEAKER_HEIGHT - shooterVertical;
    double g = FieldConstants.GRAVITY;
    double vs2 = ArmConstants.MAX_SHOOTER_VELOCITY*ArmConstants.MAX_SHOOTER_VELOCITY;
    double angle = Math.atan2(vs2 - Math.sqrt(vs2*vs2 - 2*vs2*y*g - x*x*g*g), (x*g));

    SmartDashboard.putNumberArray("xyangle", new double[] {x, y, vs2, angle});
    return 90.0 - Units.radiansToDegrees(angle);
  }

}