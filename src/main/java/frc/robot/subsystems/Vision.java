package frc.robot.subsystems;

//General imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

//Vision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.SimCameraProperties;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision extends SubsystemBase{
    private static Vision vision = null;

    //Vision objects
    private VisionSystemSim visionSim;
    private final Transform3d cameraPos;
    private AprilTagFieldLayout field;   
    private TargetModel targetModel;

    //Vision parameters
    private double t_Width = 0.5;
    private double t_Height = 0.25;

    //Advantage Scope
    private StructArrayPublisher<Pose3d> as_aprilTags;
    private StructPublisher<Pose3d> as_cameraPose; //Camera pose relative to robot on the field
    private StructPublisher<Pose2d> as_estimatedCameraPose;
    private Pose3d[] aprilTagList;

    //Pose estimation
    private PoseStrategy strat;
    private final PhotonPoseEstimator poseEstimator;

    //Camera parameters
    private SimCameraProperties cameraProp;
    private int c_Width = 640;
    private int c_Height = 480;
    private double c_Error = 0.2;
    private double fov = 70;
    private double fps = 20;
    private double latencyAve = 50; //Average latency
    private double latencySTD = 15; //Standard deviation of latency
    private Rotation2d c_FOV = Rotation2d.fromDegrees(fov);

    //Camera objects
    private PhotonCameraSim cameraSim;
    private PhotonCamera camera;

    private Vision(){
        //Field Setup
        field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        //Camera setup
        cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(c_Width, c_Height, c_FOV);
        cameraProp.setCalibError(c_Error, c_Error);
        cameraProp.setFPS(fps);
        cameraProp.setAvgLatencyMs(latencyAve);
        cameraProp.setLatencyStdDevMs(latencySTD);

        camera = new PhotonCamera("cam1");
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraPos = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5), //Position of camera on the robot
            new Rotation3d(0, 0, 0) //Rotate the camera POV
        );
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        //Simulation
        visionSim = new VisionSystemSim("main");
        visionSim.addCamera(cameraSim, cameraPos);
        visionSim.addAprilTags(field);
        targetModel = new TargetModel(t_Width, t_Height);
        
        //Pose estimation tools
        strat = PoseStrategy.AVERAGE_BEST_TARGETS;
        poseEstimator = new PhotonPoseEstimator(field, strat, cameraPos);

        //Advantage Scope
        as_aprilTags = NetworkTableInstance.getDefault().getStructArrayTopic("aprilTags", Pose3d.struct).publish();
        as_cameraPose = NetworkTableInstance.getDefault().getStructTopic("cameraPose", Pose3d.struct).publish();
        as_estimatedCameraPose = NetworkTableInstance.getDefault().getStructTopic("estimatedPose", Pose2d.struct).publish();
    }

    public void updateVision(Pose2d botPose){
        visionSim.update(botPose);
        Optional<Pose3d> camPose = visionSim.getCameraPose(cameraSim);
        as_cameraPose.set(camPose.get());
        as_aprilTags.set(field.getTags().stream().map(tag -> tag.pose).toArray(Pose3d[]::new));
    }

    public void resetVision(Pose2d resetPose){
        visionSim.resetRobotPose(resetPose);
        visionSim.clearCameras(); //Problematic line
        visionSim.clearVisionTargets(); //Problematic line
    }

    public static Vision getInstance(){
        if (vision == null){
            vision = new Vision();
        }
        return vision;
    }
}