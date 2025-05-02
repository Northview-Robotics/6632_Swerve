package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class operatorinterface extends SubsystemBase{
    private static operatorinterface oi = null;
    private XboxController controller1;
    private Drive swerve = Drive.getInstance();

    private operatorinterface(){
        controller1 = new XboxController(0);
    }

    private void updateDrive(){
        swerve.driveSwerve(controller1.getRawAxis(0), controller1.getRawAxis(1), controller1.getRawAxis(2));
       //swerve.driveSwerve( -controller1.getRawAxis(0),  -controller1.getRawAxis(1),  -controller1.getRawAxis(2));
        if (controller1.getRawButtonPressed(2)) {
            swerve.driveToPose(new Pose2d(5.0, 2.0, Rotation2d.fromDegrees(180)));
        }

        if (controller1.getRawButtonPressed(4)) {
            swerve.followDynamicPath(
                new Pose2d(1.732, 7.393, Rotation2d.fromDegrees(-43.919)),
                new Pose2d(3.964, 5.247, Rotation2d.fromDegrees(-44.648))
            );
        }
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