package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;

public class Targeting {
  SwerveSubsystem swerveSubsystem;

  public Targeting(SwerveSubsystem swerve) {
    this.swerveSubsystem = swerve;
  }

  public double getDistanceToTarget() {
    return swerveSubsystem.getPose().getTranslation().getDistance(FieldConstants.getSpeakerPosition());
  }

  public double calculateLeadingAngleOffset() {
    return 0.0f;
  }

  public double dotProd(Translation2d a, Translation2d b) {
    return (a.getX() * b.getX()) + (a.getY() * b.getY());
  }

  public Translation2d normalize(Translation2d v) {
    return v.div(v.getNorm());
  }

  public Translation2d getBotVelocity() {
    // TODO: Check that this is in the right orientation!!!
    return new Translation2d(swerveSubsystem.getChassisSpeeds().vyMetersPerSecond,
        -swerveSubsystem.getChassisSpeeds().vxMetersPerSecond).rotateBy(swerveSubsystem.getRotation2d());
  }

  public double getTargetTangentialVelocity() {
    Translation2d R = swerveSubsystem.getPose().getTranslation();
    Translation2d T = FieldConstants.getSpeakerPosition();

    // Basis vector normal to target
    Translation2d D_normal = T.minus(R);
    D_normal = D_normal.div(D_normal.getNorm());

    // Basis vector tangential to target
    Translation2d D_tangent = new Translation2d(-D_normal.getY(), -D_normal.getX());

    Translation2d bot_velocity = getBotVelocity();

    return -dotProd(D_tangent, bot_velocity);
  }

  public double getTargetNormalVelocity() {
    Translation2d R = swerveSubsystem.getPose().getTranslation();
    Translation2d T = FieldConstants.getSpeakerPosition();

    // Basis vector normal to target
    Translation2d D_normal = T.minus(R);
    D_normal = D_normal.div(D_normal.getNorm());

    Translation2d bot_velocity = getBotVelocity();
    return dotProd(D_normal, bot_velocity);
  }

  public double getPhi(double armHorizOffset) {
    // Basic aiming
    Rotation2d angle_tgt = swerveSubsystem.getPose().getTranslation().minus(FieldConstants.getSpeakerPosition())
        .getAngle();
    if (DriverStation.getAlliance().get() == Alliance.Red)
      angle_tgt = angle_tgt.rotateBy(new Rotation2d(Math.PI));

    double phi = angle_tgt.getRadians();

    SmartDashboard.putNumber("Tangentian Vel", getTargetTangentialVelocity());
    SmartDashboard.putNumber("Normal Vel", getTargetNormalVelocity());

    SmartDashboard.putString("Bot vel", getBotVelocity().toString());

    // TODO: VELOCITY COMPENSATION
    double X2 = getDistanceToTarget() + armHorizOffset;

    // Bot pos
    Translation2d R = swerveSubsystem.getPose().getTranslation();
    // Target pos
    Translation2d T = FieldConstants.getSpeakerPosition();

    Translation2d V = getBotVelocity();

    // Control forwards compensation
    // R = R.plus(V.times(0.02));

    SmartDashboard.putNumberArray("Bot velocity", new double[] { V.getX(),
        V.getY() });
    double tshotapprox = X2 / ArmConstants.MAX_SHOOTER_VELOCITY;

    Translation2d proj_shot_offset = T.plus(V.times(tshotapprox));
    double phi_correction = Math.acos(dotProd(normalize(proj_shot_offset.minus(R)),
        normalize(T.minus(R))))
        * 1.2 * Math.signum(getTargetTangentialVelocity());

    SmartDashboard.putNumber("Phi compensation", phi_correction);

    return phi;// + phi_correction;
  }

  public double getTheta(double shooterHorizontal, double shooterVertical) {
    double x = Math.max(0, getDistanceToTarget() + shooterHorizontal);

    // VELOCITY COMPENSATION
    double tshotapprox = x / ArmConstants.MAX_SHOOTER_VELOCITY;
    x -= tshotapprox * -getTargetNormalVelocity();

    double y = FieldConstants.SPEAKER_HEIGHT - shooterVertical;
    double g = FieldConstants.GRAVITY;
    double vs2 = ArmConstants.MAX_SHOOTER_VELOCITY * ArmConstants.MAX_SHOOTER_VELOCITY;
    double angle = Math.atan2(vs2 - Math.sqrt(vs2 * vs2 - 2 * vs2 * y * g - x * x * g * g), (x * g));

    SmartDashboard.putNumber("Target Distance (to shooter)", x);

    double res = 90.0 - Units.radiansToDegrees(angle);
    // SmartDashboard.putNumber("Targeting Shooter Angle", res);

    return MathUtil.clamp(res, 30, 180.0) - 2.5f;
  }

  public double getShuttlePhi() {
    // Basic aiming
    Rotation2d angle_tgt = swerveSubsystem.getPose().getTranslation().minus(FieldConstants.getShuttlePosition())
        .getAngle();
    if (DriverStation.getAlliance().get() == Alliance.Red)
      angle_tgt = angle_tgt.rotateBy(new Rotation2d(Math.PI));

    double phi = angle_tgt.getRadians();

    return phi;
  }
}
