package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class operatorinterface extends SubsystemBase{
    private static operatorinterface oi = null;
    private XboxController controller1;
    private drive drivetrain = drive.getInstance();
    private Vision vision = Vision.getInstance();
    private autoAlign align = autoAlign.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();

    private operatorinterface(){
        controller1 = new XboxController(0);
    }

    private void updateDrive(){
        drivetrain.swerveSupplier(-controller1.getLeftY(), -controller1.getLeftX(), -controller1.getRawAxis(2));
    }

    private void updateTelemetry(){
        telemetry.update();
    }

    private void updateVision(){
        vision.updateVision(drivetrain.getRobotPose());
    }

    public void updateAlign(){
        align.alignToTarget(controller1.getLeftBumperButtonReleased(), controller1.getRightBumperButtonReleased());
    }
    
    @Override
    public void periodic(){
        updateDrive();
        updateVision();
        updateAlign();
        updateTelemetry();
    }

    public static operatorinterface getInstance(){
        if (oi == null){
            oi = new operatorinterface();
        }
        return oi;
    }
}