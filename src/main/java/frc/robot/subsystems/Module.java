package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

    private RelativeEncoder turnEncoder;
    private AnalogEncoder absoluteEncoder;
    private PIDController turnPID;
    private double encoderOffset;
    private boolean encoderReversed;
    private double simulatedDistanceMeters = 0;
    private double simulatedTurningPosition = 0;

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
    }

    public void stopDrive(){
        drive.setVoltage(0);
        turn.setVoltage(0);
    }

    public double getDistance(){
        return drive.getPosition().getValueAsDouble();
    }

    public double getSimDistance(){
        return simulatedDistanceMeters;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(turnEncoder.getPosition()*(Constants.absoluteRadiansPerEncoderRotation));
    }

    public Rotation2d getSimAngle(){
        return Rotation2d.fromRadians(simulatedTurningPosition * Constants.absoluteRadiansPerEncoderRotation);
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

        //advantageScope
        simulatedDistanceMeters += state.speedMetersPerSecond* 0.02;

        simulatedDistanceMeters += state.speedMetersPerSecond * 0.02;

        // Smooth simulated turning position
        double targetAngleRad = state.angle.getRadians();
        double delta = targetAngleRad - (simulatedTurningPosition * Constants.absoluteRadiansPerEncoderRotation);

        // Wrap delta to [-π, π] for continuous input
        delta = Math.atan2(Math.sin(delta), Math.cos(delta));

        double simulatedTurnSpeed = 4.0; // Tune for sim responsiveness
        simulatedTurningPosition += delta * simulatedTurnSpeed * 0.02;

        // turnPID.setReference(state.angle.getDegrees(), ControlType.kPosition);
        // drive.setControl(driveRequest.withVelocity(state.speedMetersPerSecond));
    }
}