package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Drive extends SubsystemBase{
    private static Drive swerve = null;

    //Robot Components
    private Module rightFront;
    private Module leftFront;
    private Module rightRear;
    private Module leftRear;

    //Odometry
    private Pigeon2 gyro;

    //SwerveMath
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds chassisSpeeds;

    //Drive controls
    private double xSpeed = 0;
    private double ySpeed = 0;
    private double theta = 0;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter turnLimiter;
    private SwerveModuleState[] moduleStates;

    //Advantage scope
    private StructArrayPublisher<SwerveModuleState> publisher;
    private StructPublisher<Rotation2d> publisher2d;
    private StructPublisher<Pose3d> publisher3d;
    private StructPublisher<ChassisSpeeds> publisherSpeed;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;
    private double calcGyro = 1;
    private Rotation2d fakeHeading;

    private Drive(){
        rightFront = new Module(8,1,0, Constants.rightAbsoluteEncoderOffset, false);
        leftFront = new Module(7,2,1, Constants.leftAbsoluteEncoderOffset, false);
        rightRear = new Module(6,3,2, Constants.rightAbsoluteEncoderOffset, false);
        leftRear = new Module(5,4,3, Constants.leftAbsoluteEncoderOffset, false);
        gyro = new Pigeon2(9);
    
        //Kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
        );
        //Odometry
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(((gyro.getYaw().getValueAsDouble())*Math.PI)/180), 
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            new Pose2d(0,0,new Rotation2d()) 
        );
        //Acceleration limits
        xLimiter = new SlewRateLimiter(Constants.maxDriveAcceleration);
        yLimiter = new SlewRateLimiter(Constants.maxDriveAcceleration);
        turnLimiter = new SlewRateLimiter(Constants.maxAngularAcceleration);

        //Advantage scope
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        publisher2d = NetworkTableInstance.getDefault().getStructTopic("MyRotation", Rotation2d.struct).publish();
        publisherSpeed = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeed", ChassisSpeeds.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("/AdvantageScope/Robot/Pose", Pose3d.struct).publish();
        fakeHeading = new Rotation2d();
        // poseEstimator = new SwerveDrivePoseEstimator(kinematics, fakeHeading, getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
    }

    public void driveSwerve(double xInput, double yInput, double thetaInput){
        xSpeed = xLimiter.calculate(xInput); //Limit on X acceleration
        ySpeed = yLimiter.calculate(yInput); //Limit on Y acceletation
        theta = turnLimiter.calculate(thetaInput); //Limit on rotational acceleration
      
        if(Math.abs(xInput) < 0.1){
            xSpeed = 0;
        }
        if(Math.abs(yInput) < 0.1){
            ySpeed = 0;
        }
        if(Math.abs(thetaInput) < 0.1){
            theta = 0;
        }

        //Try only using the relative field mode if other trouble shooting methods fail
        // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, theta, new Rotation2d(gyro.getYaw().getValueAsDouble())); //Bot moves relative to the field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, -xSpeed, -theta, fakeHeading); //Bot moves relative to the field
      
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds); //Calc each module angle and speed
        setModuleStates(moduleStates); //Apply to the modules

        publisher.set(moduleStates);
        publisherSpeed.set(chassisSpeeds);
        fakeHeading = Rotation2d.fromDegrees(fakeGyro(thetaInput));
        publisher2d.set(fakeHeading);
    }

    public double fakeGyro(double joystick){
        if(joystick > 0.1){
            calcGyro = calcGyro + (5*joystick);
        }
        else if(joystick < -0.1){
            calcGyro = calcGyro + (5*joystick);
        }
        return calcGyro;
    }

    public void stopModules(){
        leftFront.stopDrive();
        rightFront.stopDrive();
        leftRear.stopDrive();
        rightRear.stopDrive();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        // kinematics.desaturateWheelSpeeds(desiredStates, Constants.gear4);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.gear4);
        leftFront.setState(desiredStates[0]);
        rightFront.setState(desiredStates[1]);
        leftRear.setState(desiredStates[2]);
        rightRear.setState(desiredStates[3]);
    }

    //Using sim methods right now
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(leftFront.getDrivingVelocity(), leftFront.getAngle()),
            new SwerveModuleState(rightFront.getDrivingVelocity(), rightFront.getAngle()),
            new SwerveModuleState(leftRear.getDrivingVelocity(), leftRear.getAngle()),
            new SwerveModuleState(rightRear.getDrivingVelocity(), rightRear.getAngle())
        };
    }
    
    //Using sim methods right now
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            new SwerveModulePosition(leftFront.getSimDistance(), leftFront.getSimAngle()), // Front-Left
            new SwerveModulePosition(rightFront.getSimDistance(), rightFront.getSimAngle()), // Front-Right
            new SwerveModulePosition(leftRear.getSimDistance(), leftRear.getSimAngle()), // Back-Left
            new SwerveModulePosition(rightRear.getSimDistance(), rightRear.getSimAngle())  // Back-Right
        };
    }

    @Override
    public void periodic(){
        // Update the odometry constantly
        // odometry.update(new Rotation2d(gyro.getYaw().getValueAsDouble()), getModulePositions());
        odometry.update(fakeHeading, getModulePositions());

        //Get current pose 3d for advantage scope
        currentPose2d = odometry.getPoseMeters();
        currentPose3d = new Pose3d(currentPose2d.getTranslation().getX(), currentPose2d.getTranslation().getY(), 0, new Rotation3d(fakeHeading));
        //Send 3D data to advantage scope
        publisher3d.set(currentPose3d);
    }

    public static Drive getInstance(){
        if (swerve == null){
            swerve = new Drive();
        }
        return swerve;
    }
}
