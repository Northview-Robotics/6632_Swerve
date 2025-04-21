package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

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
    private StructPublisher<Pose3d> publisher3d;
    private Pigeon2SimState pigeonSim;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;

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
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getYaw().getValueAsDouble()), 
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            new Pose2d(0,0,new Rotation2d()) 
        );
        //Acceleration limits
        xLimiter = new SlewRateLimiter(Constants.maxDriveAcceletation);
        yLimiter = new SlewRateLimiter(Constants.maxDriveAcceletation);
        turnLimiter = new SlewRateLimiter(Constants.maxAngularAcceletation);

        //Advantage scope
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
        pigeonSim = gyro.getSimState();
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
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, theta, new Rotation2d(gyro.getYaw().getValueAsDouble())); //Bot moves relative to the field
      

        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds); //Calc each module angle and speed
        setModuleStates(moduleStates); //Apply to the modules

        publisher.set(moduleStates);
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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(leftFront.getDrivingVelocity(), leftFront.getAngle()),
            new SwerveModuleState(rightFront.getDrivingVelocity(), rightFront.getAngle()),
            new SwerveModuleState(leftRear.getDrivingVelocity(), leftRear.getAngle()),
            new SwerveModuleState(rightRear.getDrivingVelocity(), rightRear.getAngle())
        };
    }
    

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            new SwerveModulePosition(leftFront.getDistance(), leftFront.getAngle()), // Front-Left
            new SwerveModulePosition(rightFront.getDistance(), rightFront.getAngle()), // Front-Right
            new SwerveModulePosition(leftRear.getDistance(), leftRear.getAngle()), // Back-Left
            new SwerveModulePosition(rightRear.getDistance(), rightRear.getAngle())  // Back-Right
        };
    }

    @Override
    public void periodic(){
        // Update the odometry constantly
        odometry.update(new Rotation2d(gyro.getYaw().getValueAsDouble()), getModulePositions());

        //Get current pose 3d for advantage scope
        currentPose2d = odometry.getPoseMeters();
        currentPose3d = new Pose3d(currentPose2d.getTranslation().getX(), currentPose2d.getTranslation().getY(), 0, new Rotation3d(currentPose2d.getRotation()));
        //Send 3D data to advantage scope
        publisher3d.set(currentPose3d);
    }

    //Sim
    @Override
    public void simulationPeriodic() {
        leftFront.simulationPeriodic();
        rightFront.simulationPeriodic();
        leftRear.simulationPeriodic();
        rightRear.simulationPeriodic();

        // 2) compute the “true” yaw in degrees (e.g. from your odometry or directly from theta)
        double yawDeg = odometry.getPoseMeters().getRotation().getDegrees();
        pigeonSim.setRawYaw(yawDeg);
    }

    public static Drive getInstance(){
        if (swerve == null){
            swerve = new Drive();
        }
        return swerve;
    }
}
