package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//WPILIB Imports
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//YAGSL Imports
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;


public class drive extends SubsystemBase {
    private static drive swerve = null;
    
    //YAGSL
    private File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive swerveDrive;
    private SwerveInputStream angularVel;
    private SwerveInputStream driveVel;

    //Advantage Scope
    //private StructArrayPublisher<SwerveModuleState> publisher;
    private StructPublisher<Rotation2d> publisher2d;
    private StructPublisher<Pose3d> publisher3d;
    private StructPublisher<ChassisSpeeds> publisherSpeed;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;
    private Rotation2d fakeHeading;
    private double calcGyro = 0;

    private drive(){
        try
        {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxDriveSpeed, Constants.startPose);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }

        //Advantage Scope
        //publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        publisher2d = NetworkTableInstance.getDefault().getStructTopic("MyRotation", Rotation2d.struct).publish();
        publisherSpeed = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeed", ChassisSpeeds.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("/AdvantageScope/Robot/Pose", Pose3d.struct).publish();
        fakeHeading = new Rotation2d();
    }

    public void swerveSupplier(double x, double y, double theta){
        angularVel = SwerveInputStream.of(
            returnSwerveDrive(), 
            () -> x, 
            () -> y
        ).withControllerRotationAxis(() -> theta).scaleRotation(0.8).deadband(Constants.deadband).allianceRelativeControl(true);   

        driveVel = angularVel.copy().withControllerHeadingAxis(() -> x, () ->y);
        drive(angularVel.get());
        drive(driveVel.get());

        publisherSpeed.set(swerveDrive.getFieldVelocity());
        fakeHeading = Rotation2d.fromDegrees(fakeGyro(theta));
        publisher2d.set(fakeHeading);
    }

    private SwerveDrive returnSwerveDrive(){
        return swerveDrive;
    }

    private void drive(ChassisSpeeds vel){
        swerveDrive.driveFieldOriented(vel);
    }

    public double fakeGyro(double joystick){
        if(Math.abs(joystick) > 0.1){
            calcGyro = calcGyro - (5*joystick);
        }
        return calcGyro;
    }

    //Helper methods
    public ChassisSpeeds getRobotSpeed(){
        return swerveDrive.getFieldVelocity();
    }

    public Pose2d getRobotPose(){
        return swerveDrive.getPose();
    }

    @Override
    public void periodic(){
        //Get current pose 3d for advantage scope
        currentPose2d = swerveDrive.getPose();
        currentPose3d = new Pose3d(currentPose2d.getTranslation().getX(), currentPose2d.getTranslation().getY(), 0, new Rotation3d(fakeHeading));
        //Send 3D data to advantage scope
        publisher3d.set(currentPose3d);
    }

    public static drive getInstance(){
        if (swerve == null){
            swerve = new drive();
        }
        return swerve;
    }
}