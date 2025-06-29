package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Vision extends SubsystemBase{
    public static Vision vision = null;
    private Drive swerve;
    private Intake intake;

    //Vision
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private static final Transform2d cameraOffset = new Transform2d(Constants.visionX, Constants.visionY, new Rotation2d(Constants.visionRotation)); // forward 20cm

    //Targeting
    private Pose2d currentPose;
    private Pose2d targetPose;
    private boolean reachedTarget;


    private Vision(){
        camera = new PhotonCamera("photonvision");
        result = new PhotonPipelineResult();
        swerve = Drive.getInstance();
        intake = Intake.getInstance();
    }

    public void endTrigger(){
        if(intake.checkIntake()){
            reachedTarget = false;
        }
    }

    private void getTargetPose() {
    result = camera.getLatestResult();
    if (result.hasTargets()) {
        // Get the closest target/gamepiece
        var bestTarget = result.getBestTarget();
        Transform3d target3d = bestTarget.getBestCameraToTarget();

        // Convert 3D data to 2D
        double x = target3d.getX();  
        double y = target3d.getY(); 
        Rotation2d rotation = new Rotation2d(target3d.getRotation().getZ()); // yaw only
        Transform2d cameraToTarget = new Transform2d(x, y, rotation);

        // Get robot and camera pose
        Pose2d robotPose = currentPose;
        Pose2d cameraPose = robotPose.transformBy(cameraOffset);

        // Compute field-relative target pose
        targetPose = cameraPose.transformBy(cameraToTarget);
    } else {
        targetPose = null;
        DriverStation.reportWarning("No targets...", false);
    }
}

    public void triggerIntake(boolean input1){
        if(input1){
            //Getting Targetting data
            currentPose = swerve.getPose();
            getTargetPose();
            //Targetting system
            swerve.followDynamicPath(currentPose, targetPose);
            reachedTarget = true;
            intake.autoIntake(reachedTarget);
            endTrigger();
        }
    }

    public static Vision getInstance(){
        if (vision == null){
            vision = new Vision();
        }
        return vision;
    }
}