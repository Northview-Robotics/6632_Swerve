package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class operatorinterface extends SubsystemBase{
    private static operatorinterface oi = null;
    private XboxController controller1;
    private Drive swerve = Drive.getInstance();

    private operatorinterface(){
        controller1 = new XboxController(0);
    }

    private void updateDrive(){
       swerve.driveSwerve(controller1.getLeftY(), controller1.getLeftX(), controller1.getRightX(), controller1.getYButton());
    }
    

    @Override
    public void periodic(){
        updateDrive();
    }

    public static operatorinterface getInstance(){
        if (oi == null){
            oi = new operatorinterface();
        }
        return oi;
    }
}