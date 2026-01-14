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
        //drivetrain.swerveSupplier(-controller1.getLeftY(), -controller1.getLeftX(), -controller1.getRawAxis(2));
        drivetrain.testInverts(controller1.getRightBumperButtonPressed(), controller1.getLeftBumperButtonPressed(), controller1.getYButton(), controller1.getAButton());
    }

    private void updateTelemetry(){
        telemetry.update();
    }

    private void updateVision(){
        vision.updateVision(drivetrain.getRobotPose());
    }

    public void updateAlign(){
        align.alignToTarget(controller1.getRawButtonPressed(5), controller1.getRawButtonPressed(6));
    }
    
    @Override
    public void periodic(){
        updateDrive();
        //updateVision();
        //updateAlign();
        updateTelemetry();
    }

    public static operatorinterface getInstance(){
        if (oi == null){
            oi = new operatorinterface();
        }
        return oi;
    }
}