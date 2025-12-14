package frc.robot.subsystems;

//WPILIB Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//YAGSL Imports
import java.io.File;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

//Pathplanner imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;


public class drive extends SubsystemBase {
    private static drive swerve = null;
    
    //YAGSL
    private File directory = new File(Filesystem.getDeployDirectory(),"swerve2");
    private SwerveDrive swerveDrive;
    private SwerveInputStream angularVel;
    private SwerveInputStream driveVel;

    //Advantage Scope
    private StructPublisher<Rotation2d> publisher2d;
    private StructPublisher<Pose3d> publisher3d;
    private StructPublisher<ChassisSpeeds> publisherSpeed;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;
    private Rotation2d fakeHeading;
    private double calcGyro = 0;

    //Pathplanner
    private RobotConfig config;

    private drive(){
        try
        {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxDriveSpeed, Constants.startPose);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }

        //Advantage Scope
        //publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        publisher2d = NetworkTableInstance.getDefault().getStructTopic("MyRot", Rotation2d.struct).publish();
        publisherSpeed = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeed", ChassisSpeeds.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("/AdvantageScope/Robot/Pose", Pose3d.struct).publish();
        fakeHeading = new Rotation2d();

        //Pathplanner
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Configure AutoBuilder last
            AutoBuilder.configure(
            this::getRobotPose, 
            this::resetRobotPose, 
            this::getRobotSpeed, 
            (speeds, feedforwards) -> drive(speeds), 
            new PPHolonomicDriveController( 
                    new PIDConstants(5.0, 0.0, 0.0), // Movement PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            drive.this// Reference to this subsystem to set requirements
        );

    }

    //Main driving code
    public void swerveSupplier(double x, double y, double theta){
        angularVel = SwerveInputStream.of(
            returnSwerveDrive(), 
            () -> x, 
            () -> y
        ).withControllerRotationAxis(() -> theta).scaleRotation(0.8).deadband(Constants.deadband).allianceRelativeControl(true);   

        driveVel = angularVel.copy().withControllerHeadingAxis(() -> x, () ->y);
        drive(driveVel.get());

        publisherSpeed.set(swerveDrive.getFieldVelocity());
        fakeHeading = Rotation2d.fromDegrees(fakeGyro(theta));
        publisher2d.set(fakeHeading);
    }

    //Path finding
    public void followGeneratedPath(Pose2d intermediate, Pose2d target) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getRobotPose(), intermediate, target);
        PathConstraints constraints = new PathConstraints(4.0, 3.0, 2 * Math.PI, 2 * Math.PI);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, target.getRotation())
        );
        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();  //Follow path
    }

    //Helper methods
    private SwerveDrive returnSwerveDrive(){
        return swerveDrive;
    }

    private void drive(ChassisSpeeds vel){
        swerveDrive.driveFieldOriented(vel);
    }

    public ChassisSpeeds getRobotSpeed(){
        return swerveDrive.getFieldVelocity();
    }

    public Pose2d getRobotPose(){
        return swerveDrive.getPose();
    }

    public void resetRobotPose(Pose2d pose){
        swerveDrive.resetOdometry(pose);
    }

    public double fakeGyro(double joystick){
        if(Math.abs(joystick) > 0.1){
            calcGyro = calcGyro - (5*joystick);
        }
        return calcGyro;
    }

    //Update advantage scope periodically
    @Override
    public void periodic(){
        //Get current pose 3d for advantage scope
        Rotation2d heading = swerveDrive.getYaw();
        currentPose2d = swerveDrive.getPose();
        currentPose3d = new Pose3d(currentPose2d.getX(), currentPose2d.getY(), 0, new Rotation3d(heading));
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