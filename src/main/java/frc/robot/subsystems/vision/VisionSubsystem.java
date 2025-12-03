package frc.robot.subsystems.vision;

import java.security.PublicKey;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.LimelightHelpers;
import frc.robot.library.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.PhotonAprilTagConstants.*;

import static frc.robot.RobotMap.Limelight_Front_1;

public class VisionSubsystem extends SubsystemBase{
    private final VisionState visionstste;
    private double lastTimeStampFront = 0.0;
    private Swerve swerveSubsystem;
    private final Limelight frontLimelight = new Limelight(Limelight_Front_1);
    
    private final static Set<Integer> Blue_Reef_Tags = new HashSet<>(List.of(17,18,19,20,21,22));
    private final static Set<Integer> Red_Reef_Tags  =new HashSet<>(List.of(6,7,8,9,10,11));
    private Set<Integer> currentReefTags;

    public VisionSubsystem(VisionState visionstste, Swerve swerveSubsystem){
        this.visionstste = visionstste;
        this.swerveSubsystem = swerveSubsystem;
        currentReefTags = visionstste.isRedAlliance()? Red_Reef_Tags :Blue_Reef_Tags;
    }

@Override 
public void periodic(){
    updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelight_Front_1),Limelight_Front_1);
    LimelightHelpers.SetRobotOrientation(Limelight_Front_1,visionstste.getPigeonYaw(),0,0,0,0,0);
}

public void updateVision(LimelightHelpers.PoseEstimate megaTags2Pose ,String limelightName){
    if (megaTags2Pose==null){return;}
    var updateTimeStamp = megaTags2Pose.timestampSeconds;
    boolean alreadyProccessed = false;
    switch(limelightName){
        case Limelight_Front_1:
            if(updateTimeStamp == lastTimeStampFront){
                alreadyProccessed = true;
            }
            lastTimeStampFront = updateTimeStamp;
            break;
    }

    if(!alreadyProccessed){
        Optional<VisionFieldPoseEstimate> megaTag2Estimate = processMegaTags2PoseEst(megaTags2Pose, limelightName);
    
    boolean usedMegaTag2 = false;
    if(megaTag2Estimate.isPresent()){
        if(shouldUseMegatag2(megaTags2Pose) ){
            //usedMegaTag = false;
            usedMegaTag2 = true;
            visionstste.addVisionFieldPoseEstimate(megaTag2Estimate.get());
            swerveSubsystem.addVisionMeasurement(megaTag2Estimate.get().getvisionRobotPoseMeters(), megaTag2Estimate.get().getTimestampSeconds());
        }
    }
    SmartDashboard.putBoolean("used MT2",usedMegaTag2);
}

    }

    

    public boolean shouldUseMegatag2(LimelightHelpers.PoseEstimate megaTag2Pose){

        double timestamp = megaTag2Pose.timestampSeconds;
        var angularYawVelo = visionstste.getGreatestAngularYawVelocity(timestamp- 0.1,timestamp);
        if (angularYawVelo.isPresent() && Math.abs(angularYawVelo.get())>Units.degreesToRadians(200)){
            return false;
        }

        return true;
    }

    public Optional<Pose2d> getFieldToRobot(LimelightHelpers.PoseEstimate PoseEstimate,String LimelightName){
        var FieldToCamera = PoseEstimate.pose;
        if(FieldToCamera.getX() == 0)return Optional.empty();

        return Optional.of(FieldToCamera); 
    }

    public Optional<VisionFieldPoseEstimate> processMegaTags2PoseEst(LimelightHelpers.PoseEstimate poseEstimate,String LimelightName){
        double timestamp = poseEstimate.timestampSeconds;
        var realFieldToRobot = visionstste.getFieldToRobot(timestamp);
        if(realFieldToRobot.isEmpty())return Optional.empty();

        var estFieldToRobot = getFieldToRobot(poseEstimate, LimelightName);
        if(estFieldToRobot.isEmpty())return Optional.empty();

        double poseDifference = estFieldToRobot.get().getTranslation().getDistance(realFieldToRobot.get().getTranslation());

        var estStdDevs = Constants.PhotonAprilTagConstants.kSingleTagStdDevs;

        if(poseEstimate.rawFiducials.length  > 0){
            
            double xyStdDev = 2.0;  
            double thetaStdDev = Units.degreesToRadians(50.0);
            double avgDist = poseEstimate.avgTagDist;
            


            if(poseEstimate.rawFiducials.length == 1) {
                xyStdDev = Constants.PhotonAprilTagConstants.xyStdDevModel.predict(avgDist);
                thetaStdDev = Constants.PhotonAprilTagConstants.thetaStdDevModel.predict(avgDist);
                estStdDevs = VecBuilder.fill(xyStdDev*59.4, xyStdDev*59.4, thetaStdDev*59.4);
            }
            else if(poseEstimate.rawFiducials.length > 1) {
                xyStdDev = Math.pow(avgDist, 2.0) / poseEstimate.rawFiducials.length;
                thetaStdDev = Math.pow(avgDist, 2.0) / poseEstimate.rawFiducials.length;
                estStdDevs = VecBuilder.fill(xyStdDev*59.4, xyStdDev*59.4, thetaStdDev*59.4);
            }

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
            
            Pose2d fieldToRobotEstimate = new Pose2d(estFieldToRobot.get().getTranslation(), realFieldToRobot.get().getRotation());
            
            return Optional.of(new VisionFieldPoseEstimate(fieldToRobotEstimate, timestamp, visionMeasurementStdDevs));
        }
            return Optional.empty();
    }

    private boolean seesReefTag(RawFiducial[] fids){
        for(int tag: currentReefTags)        
            for(RawFiducial fid: fids){    
                if(tag == fid.id){
                    return true;
                }
            }

        return false;
    }

    public boolean getTv(){
        return LimelightHelpers.getTV(Limelight_Front_1);
    }
        /** 給瞄準 / 射擊 command 用的：有沒有看到任何 target */
    public boolean hasTarget() {
        return frontLimelight.getTv();
    }
    
        /** 給瞄準 / 射擊 command 用的：目前目標在畫面水平偏移多少度 */
    public double getTx() {
        return frontLimelight.getTx();
    }
    public double getTy(){
        return LimelightHelpers.getTY(Limelight_Front_1);
    }

    public int getID(){
        if(getTv()){
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelight_Front_1).rawFiducials[0].id;
        }
    
        return -1;
    }
}