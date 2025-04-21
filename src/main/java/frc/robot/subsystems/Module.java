package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Module extends SubsystemBase{
    private SparkMax turn;
    private TalonFX drive;
    private SparkMaxConfig config1;

    private RelativeEncoder turnEncoder;
    private AnalogEncoder absoluteEncoder;
    private PIDController turnPID;
    private double encoderOffset;
    private boolean encoderReversed;

    //Advantage scope
    private TalonFXSimState driveSim;

    public Module(int driveID, int turnID, int absoluteEncoderID, double moduleOffset, boolean reversed){
        //Module initializations
        turn = new SparkMax(turnID, MotorType.kBrushless);
        drive = new TalonFX(driveID);

        //Encoders
        absoluteEncoder = new AnalogEncoder(absoluteEncoderID, Constants.absoluteRadiansPerEncoderRotation,0);
        encoderOffset = moduleOffset;
        encoderReversed = reversed;
        turnEncoder = turn.getEncoder();

        //Motor Configurations
        // turn.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        //PIDS
        turnPID = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

       //Advantage scope
       driveSim = drive.getSimState();
    }

    public void stopDrive(){
        drive.setVoltage(0);
        turn.setVoltage(0);
    }

    public double getDistance(){
        return drive.getPosition().getValueAsDouble();
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(turnEncoder.getPosition()*(Constants.absoluteRadiansPerEncoderRotation));
    }

    public double getTurningPosition(){
        return turnEncoder.getPosition();
    }   

    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }   

    public double getDrivingVelocity(){
        return ((drive.getVelocity().getValueAsDouble())*12.566);
    }

    public void resetEncoder(){
        drive.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoder());
    }

    public double getAbsoluteEncoder(){
        double angle = absoluteEncoder.get();
        angle *= 2.0*Math.PI;
        angle -= encoderOffset;
        
        return angle * (encoderReversed ? -1.0 : 1.0);
    }
        
    public void setState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stopDrive();
            return;
        }

        // state.optimize(new Rotation2d(getAbsoluteEncoder()));
        state.optimize(new Rotation2d(getAbsoluteEncoder()));
        drive.set(state.speedMetersPerSecond/ Constants.gear4); 
        turn.setVoltage(turnPID.calculate(getTurningPosition(), state.angle.getRadians()));
        // turnPID.setReference(state.angle.getDegrees(), ControlType.kPosition);
        // drive.setControl(driveRequest.withVelocity(state.speedMetersPerSecond));
    }

    //Sim
    @Override
    public void simulationPeriodic() {
      // 1) Read your commanded wheel speed (m/s)
      double wheelSpeedMps = getDrivingVelocity();
  
      // 2) Convert to rotations per second:
      double wheelCircumference = Constants.wheelDiameterMeters * Math.PI;
      double rps = wheelSpeedMps / wheelCircumference;
  
      // 3) Tell the TalonFX sim what its rotor velocity should be
      driveSim.setRotorVelocity(rps);
    }
}