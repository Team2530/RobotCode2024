package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  public enum Presets {
    // Stage one angle is 0 refrenced from the horizontal
    // stage 2 angle is refrenced as zero relative to stage one, intake pointing
    // out the front when the arm is vertical, and the intake horizontal
    STOW(-0.37, 181),
    SHOOT_LOW(0, -40.6),
    INTAKE(-14.7, 42.7),
    AMP(100, 134),
    SHOOT_HIGH(22, 58),
    STARTING_CONFIG(0, 90),
    DYNAMIC(0,0);

    private double s1angle;
    private double s2angle;

    private Presets(double s1angle, double s2angle) {
      this.s1angle = s1angle;
      this.s2angle = s2angle;
    }

  }

  private final StageOne stageOne;
  private final StageTwo stageTwo;

  private Presets currentPreset = Presets.STARTING_CONFIG;

  public Arm(StageOne stageOne, StageTwo stageTwo) {
    this.stageOne = stageOne;
    this.stageTwo = stageTwo;
  }

  @Override
  public void periodic() {
    stageTwo.updateStageOneOffset(stageOne.getMeasurement());

    SmartDashboard.putNumber("Stage One Encoder", Units.radiansToDegrees(stageOne.getMeasurement()));
    SmartDashboard.putNumber("Stage Two Encoder", Units.radiansToDegrees(stageTwo.getMeasurement()));
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

  public void setArmAngles(double shoulder, double wrist) {
    stageOne.setGoalDegrees(shoulder);
    stageTwo.setGoalDegrees(wrist);
    currentPreset = Presets.DYNAMIC;
    SmartDashboard.putString("Arm Preset", "Moving to DYNAMIC, 1: " + shoulder + " 2: " + wrist);
  }

}