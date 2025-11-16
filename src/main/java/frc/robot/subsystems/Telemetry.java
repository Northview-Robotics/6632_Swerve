package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogEncoder;
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
    Pigeon2 pigeon;

    //lF module encoder, rF module encoder, rR module encoder, lR module encoder
    AnalogEncoder encoder1;
    AnalogEncoder encoder2;
    AnalogEncoder encoder3;
    AnalogEncoder encoder4;

    private Telemetry(){
        pdp = new PowerDistribution(10, ModuleType.kCTRE);
        field2d = new Field2d();
        //Send field2d to the dashboard
        SmartDashboard.putData("Field ", field2d);
        pigeon = new Pigeon2(9);
        encoder1 = new AnalogEncoder(0);
        encoder2 = new AnalogEncoder(1);
        encoder3 = new AnalogEncoder(2);
        encoder4 = new AnalogEncoder(3);
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

        //Send current yaw to the dashboard
        double yaw = pigeon.getYaw().getValueAsDouble();
        SmartDashboard.putNumber("Yaw ", yaw);

        //Send encoder values to the dashboard
        double encVal1 = encoder1.get();
        SmartDashboard.putNumber("Encoder 1 ", encVal1);

        double encVal2 = encoder1.get();
        SmartDashboard.putNumber("Encoder 2 ", encVal2);

        double encVal3 = encoder1.get();
        SmartDashboard.putNumber("Encoder 3 ", encVal3);

        double encVal4 = encoder1.get();
        SmartDashboard.putNumber("Encoder 4 ", encVal4);

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
