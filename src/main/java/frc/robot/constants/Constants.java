package frc.robot.constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final Translation2d robotCenter = new Translation2d(0, 0); // serves as "center of robot for calculations; robot will turn about this point
  public static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(5.5), -Units.inchesToMeters(5.5)); // units in meters
  public static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(5.5));
  public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(robotCenter.minus(leftPodPosition), robotCenter.minus(rightPodPosition));

  public static final double leftAbsoluteEncoderOffset = 1.006118384272561;//absolute encoder reading at position
  public static final double rightAbsoluteEncoderOffset = 1.056659935134538;// gears facing inwards: fwd/bck 

  public static final double turnRadiansPerMotorRotation = 2*Math.PI*15/33;
  public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI * 33 / 45 / 15/13;
  public static final double absoluteRadiansPerEncoderRotation = 2*Math.PI;
  public static final double turnDriveSpeedMultiplier = 0.5;//0;

  //Gyro settings
  public static final double gyroP = 2;
  public static final double gyroI = 0.0;
  public static final double gyroD = 0.00;

  //Drive settings
  public static final int driveAmpLimit= 50;
  public static final double gear1 = 2;
  public static final double gear2 = 3;
  public static final double gear3 = 4;
  public static final double gear4 = 5;
  public static final double driveSpeedToPower = 1.0;
  public static final double driveMotorRampRate = 0.2;
  public static final IdleMode driveMode = IdleMode.kCoast;
  public static final double podMaxSpeed = 1;

  //Steer settings
  public static final double turnP = 0.4;
  public static final double turnI = 0.0;
  public static final double turnD = 0.003;
  public static final int turnAmpLimit = 55;
  public static final IdleMode turnMode = IdleMode.kBrake;
}