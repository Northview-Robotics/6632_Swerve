package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.autoConstants;

public class autoAlign extends SubsystemBase{
    private static autoAlign align = null;
    private drive drivetrain = drive.getInstance();
    private Vision vision = Vision.getInstance();

    public void alignToTarget(boolean input1, boolean input2){
        int targetTag = vision.getTargetAprilTag();
        
        switch(targetTag){
            case autoConstants.tagB1:
                if(input1) drivetrain.followGeneratedPath(autoConstants.B1LI, autoConstants.B1L);
                if(input2) drivetrain.followGeneratedPath(autoConstants.B1RI, autoConstants.B1R);
                break;
            case autoConstants.tagB2:
                if(input1) drivetrain.followGeneratedPath(autoConstants.B2LI, autoConstants.B2L);
                if(input2) drivetrain.followGeneratedPath(autoConstants.B2RI, autoConstants.B2R);
                break;
            case autoConstants.tagB3:
                if(input1) drivetrain.followGeneratedPath(autoConstants.B3LI, autoConstants.B3L);
                if(input2) drivetrain.followGeneratedPath(autoConstants.B3RI, autoConstants.B3R);
                break;
            case autoConstants.tagB4:
                if(input1) drivetrain.followGeneratedPath(autoConstants.B4LI, autoConstants.B4L);
                if(input2) drivetrain.followGeneratedPath(autoConstants.B4RI, autoConstants.B4R);
                break;
            case autoConstants.tagB5:
                if(input1) drivetrain.followGeneratedPath(autoConstants.B5LI, autoConstants.B5L);
                if(input2) drivetrain.followGeneratedPath(autoConstants.B5RI, autoConstants.B5R);
                break;
            case autoConstants.tagB6:
                if(input1) drivetrain.followGeneratedPath(autoConstants.B6LI, autoConstants.B6L);
                if(input2) drivetrain.followGeneratedPath(autoConstants.B6RI, autoConstants.B6R);
                break;
        }
    }

    public static autoAlign getInstance(){
        if (align == null){
            align = new autoAlign();
        }
        return align;
    }
}
