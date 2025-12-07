package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

// 只負責「靠近 AprilTag 到 15cm」，不改機器人旋轉角度
public class CloseTagCommand extends Command {
    private final Swerve drivetrain;
    private final VisionSubsystem vision;
    private final CommandXboxController driver; // 目前沒用，但保留以後擴充

    private final SwerveRequest.FieldCentric driveRequest;

    // 底盤參數
    private double maxSpeed;

    // ---- 相機 / Tag 幾何 ----
    // 相機離地高度 (m)
    private static final double CAMERA_HEIGHT_M = 0.195;  // 19.5 cm
    // Tag 中心離地高度 (m)
    private static final double TARGET_HEIGHT_M = 0.300;  // 30 cm
    // 相機往上仰角
    private static final Rotation2d CAMERA_PITCH_UP =
        Rotation2d.fromDegrees(23.0);
    // 希望離 Tag 平面距離 (m)
    private static final double TARGET_DISTANCE_M = 0.150; // 15 cm

    // 距離控制的 P，實機要調
    private static final double kP_DISTANCE = 1.0;
    // 自動靠近時最高前進速度比例（不要用滿 MaxSpeed，會太猛）
    private static final double AUTO_SPEED_RATIO = 0.5;
    // 認為「已到位」的距離誤差（m）
    private static final double DIST_TOLERANCE_M = 0.02;  // 2 cm

    public CloseTagCommand(
        Swerve drivetrain,
        VisionSubsystem vision,
        CommandXboxController driver,
        double maxSpeed
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driver = driver;
        this.maxSpeed = maxSpeed;

        addRequirements(drivetrain);

        // 跟你其他地方一樣，用 FieldCentric request
        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(0.0) // 這個指令不控制旋轉
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    @Override
    public void initialize() {
        // 這個 command 沒有自己的 PID 狀態要清，先空著
    }

    @Override
    public void execute() {
    double forwardSpeed = 0.0;

    // 先準備幾個要顯示的變數
    boolean hasTarget = vision.hasTarget();
    double tyDeg = 0.0;
    double distance = 0.0;
    double distError = 0.0;

    if (!hasTarget) {
        forwardSpeed = 0.0;
    } else {
        tyDeg = vision.getTy();  // 垂直偏移角 (deg)

        // 總仰角 = 安裝角 + ty
        double totalAngleRad =
            CAMERA_PITCH_UP.getRadians() + Math.toRadians(tyDeg);

        // 避免 tan(0) 炸掉：角度太小就暫時不動
        if (Math.abs(totalAngleRad) > Math.toRadians(1.0)) { // >1°
            distance =
                (TARGET_HEIGHT_M - CAMERA_HEIGHT_M) / Math.tan(totalAngleRad);

            distError = distance - TARGET_DISTANCE_M; // 正 = 太遠，負 = 太近

            // 用 P 把距離誤差變成速度
            double rawSpeed = kP_DISTANCE * distError;
            // 如果方向反了，就改成：double rawSpeed = -kP_DISTANCE * distError;

            double maxAutoSpeed = maxSpeed * AUTO_SPEED_RATIO;
            forwardSpeed = MathUtil.clamp(rawSpeed, -maxAutoSpeed, maxAutoSpeed);

            // 靠近到誤差 < 2cm 就停下來
            if (Math.abs(distError) < DIST_TOLERANCE_M) {
                forwardSpeed = 0.0;
            }

        } else {
            // 幾乎水平（很遠或誤差太大），先不要亂跑
            forwardSpeed = 0.0;
        }
    }

    // ======= 把資料丟到 SmartDashboard =======
    SmartDashboard.putBoolean("CloseTag/HasTarget", hasTarget);
    SmartDashboard.putNumber("CloseTag/tyDeg", tyDeg);
    SmartDashboard.putNumber("CloseTag/DistanceMeters", distance);
    SmartDashboard.putNumber("CloseTag/DistError", distError);
    SmartDashboard.putNumber("CloseTag/ForwardSpeed", forwardSpeed);

    // ======= 發送到底盤 =======
    drivetrain.setControl(
        driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
    );

    }

    @Override
    public void end(boolean interrupted) {
        // 放開按鈕 / 指令結束時停下來
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        // 用 whileTrue 控制；按住就跑、放開就結束
        return false;
    }
}
