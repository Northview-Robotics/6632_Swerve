package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase{
    private static Telemetry telemetry = null;
    private PowerDistribution pdp;
    private Field2d field2d;
    private drive drivetrain = drive.getInstance();

    private Telemetry(){
        pdp = new PowerDistribution(10, ModuleType.kCTRE);
        field2d = new Field2d();
        //Send field2d to the dashboard
        SmartDashboard.putData("Field ", field2d);
    }

    public void update(){
        //Send current velocity to the dashboard
        ChassisSpeeds robotSpeed = drivetrain.getRobotSpeed();
        double robotVelocity = Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        SmartDashboard.putNumber("Velocity", robotVelocity);

        //Send current rotational veloctity to the dashboard
        double rotationalVelocity = robotSpeed.omegaRadiansPerSecond;
        SmartDashboard.putNumber("Rotation ", rotationalVelocity);

        //Send match time to the dashboard
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        
        //Send current voltage to the dashboard
        double voltage = pdp.getVoltage();
        SmartDashboard.putNumber("Voltage ", voltage);

        //Update robot position on the 2d field inside the dashboard
        field2d.setRobotPose(drivetrain.getRobotPose());
    }

    public static Telemetry getInstance(){
        if (telemetry == null){
            telemetry = new Telemetry();
        }
        return telemetry;
    }
}
