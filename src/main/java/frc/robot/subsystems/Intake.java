package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase{
    private static Intake intake = null;
    //Motor initializations
    private SparkMax driverMotor1;
    private SparkMax driverMotor2;
    private SparkMax intakeMotor;

    //Initializing beam breaks
    private DigitalInput driverBeam;
    private DigitalInput holdingBeam;

    boolean driverBreak;
    boolean holdingBreak;

    private Intake(){
        //Motors
        driverMotor1 = new SparkMax(9, MotorType.kBrushless);
        driverMotor2 = new SparkMax(10, MotorType.kBrushless);
        intakeMotor = new SparkMax(11, MotorType.kBrushless);
        //Beam break
        driverBeam = new DigitalInput(0);
        holdingBeam = new DigitalInput(1);
    }

    //Manual operation of the intake
    public void runIntake(double forward, double backward){
        if(forward > 0.1){
            driverMotor1.set(0.5);
            driverMotor2.set(-0.5);
            intakeMotor.set(0.5);
        }
        else if(backward > 0.1){
            driverMotor1.set(-0.5);
            driverMotor2.set(0.5);
            intakeMotor.set(-0.5);
        }
        else{
            driverMotor1.set(0);
            driverMotor2.set(0);
            intakeMotor.set(0);
        }
    }

    public void stopIntake(){
        driverMotor1.set(0);
        driverMotor2.set(0);
        intakeMotor.set(0);
    }

    public void stopDrivers(){
        driverMotor1.set(0);
        driverMotor2.set(0);
    }

    public void stopMain(){
        intakeMotor.set(0);
    }

    //Automatic functioning of the intake
    public void runDrivers(double speed){
        driverMotor1.set(speed);
        driverMotor2.set(-speed);
    }   

    public void runMain(double speed){
        intakeMotor.set(speed);
    }

    public void autoIntake(boolean target){
        if(target){
           if(!holdingBreak){
                if(!driverBreak){
                    runDrivers(-0.5);
                    runMain(-0.5);
                }
                stopIntake();
           }
           else{
                runMain(0.5);
                if(!driverBreak){
                    runDrivers(0.5);
                    stopMain();
                }
           }
        }
        else{
            stopIntake();
        }
    }

    public boolean checkIntake(){
        if(!holdingBreak){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void periodic() {
        driverBreak = driverBeam.get();
        holdingBreak = holdingBeam.get();
    }

    public static Intake getInstance(){
        if (intake == null){
            intake = new Intake();
        }
        return intake;
    }
}