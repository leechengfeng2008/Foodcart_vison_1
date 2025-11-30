package frc.robot.subsystems.vision;

import frc.robot.library.LimelightHelpers;

public class Limelight {
    private String limelightName;
    public Limelight(String limelightName){
        this.limelightName = limelightName;
    }

    public double getTx(){
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTy(){
        return LimelightHelpers.getTY(limelightName);
    }

    public int getID(){
        if(getTv()){
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName).rawFiducials[0].id;
        }
    
        return -1;
    }

    public boolean getTv(){
        return LimelightHelpers.getTV(limelightName);
    }

    public void setLEDMode(boolean enabled){
        if(enabled){
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        }
        else{
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        }
    }
}
