package frc.robot.subsystems;

import java.lang.annotation.Target;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Targeting;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Shooter.ShooterMode;

public class Arm extends SubsystemBase {

  public enum Presets {
    // Stage one angle is 0 refrenced from the horizontal
    // stage 2 angle is refrenced as zero relative to stage one, intake pointing
    // out the front when the arm is vertical, and the intake horizontal
    STOW(-0.37, 181),
    SHOOT_LOW(19, 48),
    SHOOT_TM(19, 38),
    INTAKE(-14.7, 34.5),
    AMP(101, 125),    AUTO_AMP(101, 120),

    SHOOT_HIGH(90, 40),
    SHOOT_MANUAL(19, 48),
    SHOOT_SHUTTLE(19, 48),
    STARTING_CONFIG(0, 90),
    SOURCE(42, 132),
    // TRAP(33, 47),
    TRAP(90, 36),
    //
    CLIMB(-14.7, 74),
    CUSTOM(0, 0);

    private double s1angle;
    private double s2angle;

    private Presets(double s1angle, double s2angle) {
      this.s1angle = s1angle;
      this.s2angle = s2angle;
    }

  }

  private final StageOne stageOne;
  public final StageTwo stageTwo;
  private final XboxController operatorXbox;
  private final Shooter shooter;
  private Targeting targeting;

  private Presets currentPreset = Presets.STARTING_CONFIG;

  public Arm(StageOne stageOne, StageTwo stageTwo, Shooter shooter, Targeting targeting, XboxController operatorXbox) {
    this.stageOne = stageOne;
    this.stageTwo = stageTwo;
    this.operatorXbox = operatorXbox;
    this.shooter = shooter;
    this.targeting = targeting;
  }

  public void hardwareInit() {
    stageOne.hardwareInit();
    stageTwo.hardwareInit();
  }

  @Override
  public void periodic() {
    stageTwo.updateStageOneOffset(stageOne.getMeasurement());

    SmartDashboard.putNumber("Stage One Encoder", Units.radiansToDegrees(stageOne.getMeasurement()));
    SmartDashboard.putNumber("Stage Two Encoder", Units.radiansToDegrees(stageTwo.getMeasurement()));

    if (currentPreset == Presets.SHOOT_LOW) {
      stageTwo.setGoalDegrees(targeting.getTheta(ArmConstants.SHOOTER_LOW_X_OFFSET, ArmConstants.SHOOTER_LOW_HEIGHT));
    } else if (currentPreset == Presets.SHOOT_HIGH) {
      stageTwo.setGoalDegrees(targeting.getTheta(ArmConstants.SHOOTER_HIGH_X_OFFSET, ArmConstants.SHOOTER_HIGH_HEIGHT));
    }
  }

  public void setArmPreset(Presets preset) {
    // only change if new goal
    if (currentPreset != preset) {

      if (DriverStation.isEnabled()) {
        // If moving from intake to stow, only cam up the intake, don't use stage one at
        // all
        if (currentPreset == Presets.INTAKE && preset == Presets.STOW) {
          stageOne.coast();
          stageOne.disable();
          stageTwo.enable();
        } else {
          stageOne.brake();
          stageOne.enable();
          stageTwo.enable();
        }
      }

      stageTwo.setGoalDegrees(preset.s2angle);
      stageOne.setGoalDegrees(preset.s1angle);
      SmartDashboard.putString("Arm Preset", "Moving to " + currentPreset.name());

      currentPreset = preset;

      if (shooter.getShooterMode() != ShooterMode.STOPPED && operatorXbox.getRightBumper()) {
        shooter.setCustomPercent(getPresetShooterSpeed());
      }

    }
  }

  /**
   * Sets the arm to a custom goal for stage 1 and stage 2
   * 
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
        return 0.85;
      case SHOOT_LOW:
        return 0.85;
      case SHOOT_MANUAL:
        return 0.85;
      case SHOOT_SHUTTLE:
        return 0.70;
      case AMP:
        return 0.5;
      case TRAP:
        return 0.65;
      default:
        return 0.5;
    }
  }

  public double getStageOneDegrees() {
    return stageOne.getMeasurement();
  }

  public double getHorizOffset() {
    if (currentPreset == Presets.SHOOT_LOW) {
      return ArmConstants.SHOOTER_LOW_X_OFFSET;
    } else if (currentPreset == Presets.SHOOT_HIGH) {
      return ArmConstants.SHOOTER_HIGH_X_OFFSET;
    } else {
      return 0.0;
    }
  }

  public boolean isShooterAligned() {
    return Math.abs(getStageTwoDegrees() - stageTwo.getGoal()) < 1.25;
  }

  public Presets getCurrentPreset() {
    return currentPreset;
  }

  public double getStageTwoDegrees() {
    return stageTwo.getMeasurement();
  }
}