package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class operatorinterface extends SubsystemBase{
    private static operatorinterface oi = null;
    private XboxController controller1;
    private Drive swerve = Drive.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();

    private operatorinterface(){
        controller1 = new XboxController(0);
    }

    private void updateDrive(){
        swerve.driveSwerve(controller1.getRawAxis(0), controller1.getRawAxis(1), controller1.getRawAxis(2));
        swerve.chooseTarget(controller1.getRightBumperButtonPressed(), controller1.getLeftBumperButtonPressed(), controller1.getYButtonPressed(), controller1.getRawButtonPressed(3));
    }

    private void updateTelemetry(){
        telemetry.update();
    }
    
    @Override
    public void periodic(){
        updateDrive();
        updateTelemetry();
    }

    public static operatorinterface getInstance(){
        if (oi == null){
            oi = new operatorinterface();
        }
        return oi;
    }
}