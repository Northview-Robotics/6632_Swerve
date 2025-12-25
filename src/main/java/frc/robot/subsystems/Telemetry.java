package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Telemetry extends SubsystemBase{
    private static Telemetry telemetry = null;
    private Field2d field2d;
    private drive drivetrain = drive.getInstance();

    private Telemetry(){
        field2d = new Field2d();
        SmartDashboard.putData("Field ", field2d);
    }

    public void update(){
        //Send match time to the dashboard
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        //Update robot position on the 2d field inside the dashboard
        field2d.setRobotPose(drivetrain.getRobotPose());

        //Update robot driving velocity
        ChassisSpeeds currentVelocity = drivetrain.getRobotSpeed();
        double botVelocity = Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
        SmartDashboard.putNumber("Velocity", botVelocity);
    }

    public static Telemetry getInstance(){
        if (telemetry == null){
            telemetry = new Telemetry();
        }
        return telemetry;
    }
}
