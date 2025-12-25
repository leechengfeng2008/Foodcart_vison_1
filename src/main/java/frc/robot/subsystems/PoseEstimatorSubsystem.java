package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final Swerve m_swerve;
    
    // Field2d 用於在 Dashboard 上顯示機器人位置
    private final Field2d m_field = new Field2d();

    public PoseEstimatorSubsystem(Swerve swerve) {
        this.m_swerve = swerve;
        // 初始化時將 Field 放入 SmartDashboard
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        // 1. 更新內部視覺數據 (MegaTag1 範例)
        updateVisionMeasurement();

        // 2. 獲取目前整合後的 Pose (從 Swerve 內置的 Estimator 拿取)
        Pose2d currentPose = m_swerve.getState().Pose;

        // 3. 更新 Dashboard 上的圖形顯示
        m_field.setRobotPose(currentPose);
        
        // 4. 輸出數值以便偵錯
        SmartDashboard.putNumber("Robot/PoseX", currentPose.getX());
        SmartDashboard.putNumber("Robot/PoseY", currentPose.getY());
        SmartDashboard.putNumber("Robot/PoseDegrees", currentPose.getRotation().getDegrees());
        Logger.recordOutput("PoseEstimator/Pose", currentPose);
    }

    /**
     * 從 Limelight 讀取 MegaTag1 數據並推送到 Swerve 的 PoseEstimator
     */
    private void updateVisionMeasurement() {
        // 取得 Limelight 的 botpose_wpiblue (會自動處理聯盟顏色座標)
        double[] botpose = NetworkTableInstance.getDefault()
            .getTable("limelight-front")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[0]);

        // 檢查：陣列長度足夠 且 至少看到一個 Tag (index 7 是 Tag 數量)
        if (botpose.length >= 8 && botpose[7] > 0) {
            double x = botpose[0];
            double y = botpose[1];
            double thetaDegrees = botpose[5];
            double latencyMs = botpose[6]; // 總延遲 (影像 + 傳輸)

            Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees));
            Logger.recordOutput("VisionPose", visionPose);
            // 計算正確的數據採樣時間點
            double timestampSeconds = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

            // 調用你 Swerve 類別中的 addVisionMeasurement
            // 此方法會透過卡爾曼濾波器自動融合數據
            m_swerve.addVisionMeasurement(visionPose, timestampSeconds,VecBuilder.fill(0.01, 0.01, 1));
        }
    }

    /**
     * 提供給 PathPlanner 或自動程式使用的介面
     */
    public Pose2d getCurrentPose() {
        return m_swerve.getState().Pose;
    }

    /**
     * 重設目前的場地座標
     */
    public void setCurrentPose(Pose2d pose) {
        m_swerve.resetPose(pose);
    }
}