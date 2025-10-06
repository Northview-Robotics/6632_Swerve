package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    public static Elevator elevator = null;
    private TalonFX rightLift;
    private TalonFX leftLift;

    private Elevator(){
        rightLift = new TalonFX(12);
        leftLift = new TalonFX(13);
    }

    public static Elevator getInstance(){
        if (elevator == null){
            elevator = new Elevator();
        }
        return elevator;
    }
}
