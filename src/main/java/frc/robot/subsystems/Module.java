package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
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
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Module extends SubsystemBase{
    private SparkMax turn;
    private TalonFX drive;
    private SparkMaxConfig config1;

    private RelativeEncoder turnEncoder;
    private AnalogEncoder absoluteEncoder;
    private PIDController turnPID;
    private double encoderOffset;


    public Module(int driveID, int turnID, int absoluteEncoderID, double moduleOffset, boolean encoderReversed){
        //Module initializations
        turn = new SparkMax(turnID, MotorType.kBrushless);
        drive = new TalonFX(driveID);

        //Encoders
        absoluteEncoder = new AnalogEncoder(absoluteEncoderID, Constants.absoluteRadiansPerEncoderRotation,0);
        encoderOffset = moduleOffset;
        turnEncoder = turn.getEncoder();

        //Motor Configurations
        turn.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //PIDS
        turnPID = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

       
    }

    public void stopDrive(){
        drive.setVoltage(0);
        turn.setVoltage(0);
    }

    public double getDistance(){
        return drive.getPosition().getValueAsDouble();
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(turnEncoder.getPosition());
    }

    public double getTurningPosition(){
        return turnEncoder.getPosition();
    }   

    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }   

    public void resetEncoder(){
        drive.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoder());
    }

    public double getAbsoluteEncoder(){
        double angle = absoluteEncoder.get();
        angle *= 2.0*Math.PI;
        angle -= encoderOffset;

        return angle;
    }
    

    public void setState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stopDrive();
            return;
        }

        state = SwerveModuleState.optimize(state, getAbsoluteEncoder());
        drive.set(state.speedMetersPerSecond/ Constants.gear4); 
        turn.setVoltage(turnPID.calculate(getTurningPosition(), state.angle.getRadians()));
        // turnPID.setReference(state.angle.getDegrees(), ControlType.kPosition);
        // drive.setControl(driveRequest.withVelocity(state.speedMetersPerSecond));
    }
}