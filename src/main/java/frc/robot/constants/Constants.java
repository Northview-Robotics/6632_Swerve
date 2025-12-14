package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
  //YAGSL
  public static final double maxDriveSpeed = Units.feetToMeters(16);
  public static final Pose2d startPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
  public static final double deadband = 0.05;

  //Vision 
  public static final double visionX = 0.2;
  public static final double visionY = 0.0;
  public static final double visionRotation = 0.0; //
}