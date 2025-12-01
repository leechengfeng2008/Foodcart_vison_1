package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants_Foodcart;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;

public class AlignCommand extends Command {
    private final Swerve drivetrain;
    private final VisionSubsystem limelight;
    private final CommandXboxController driver;

    private final SwerveRequest.FieldCentric driveRequest;

    private final PIDController thetaPID;

  // 最大線速度（公尺/秒）
  private double maxSpeed = TunerConstants_Foodcart.kSpeedAt12Volts.in(MetersPerSecond); 
  // 最大角速度（弧度/秒）
  private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    public AlignCommand(
        Swerve drivetrain,
        VisionSubsystem limelight,
        CommandXboxController driver,
        double maxSpeed,
        double maxAngularRate
    ) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.driver = driver;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        // 這個 Command 要控制底盤
        addRequirements(drivetrain);

        // 跟你 RobotContainer 裡的 drive 一樣型態的 request
        driveRequest = new SwerveRequest.FieldCentric()
        // 線速度和角速度都加上 10% 的死區
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


        // 這顆 PID 專門鎖 tx
        // 先隨便給一個 P，之後你再用常數或 SmartDashboard 調
        thetaPID = new PIDController(0.02, 0.0, 0.0);
        thetaPID.setSetpoint(0.0);   // 希望 tx = 0
        thetaPID.setTolerance(1.0);  // 接受 ±1 度內視為鎖好
    }

    @Override
    public void initialize() {
        thetaPID.reset();
    }

    @Override
    public void execute() {
        // 1. 平移一樣照搖桿（你現在 default drive 的寫法）
        double xSpeed = -driver.getLeftY() * maxSpeed;
        double ySpeed = -driver.getLeftX() * maxSpeed;

        // 2. 先預設旋轉照搖桿，沒目標時就當一般開車
        double omega = -driver.getRightX() * maxAngularRate;

        // 3. 如果 Limelight 有看到目標，就用 PID 鎖 tx
        if (limelight.hasTarget()) {
            double tx = limelight.getTx();

            // measurement = tx, setpoint 在 constructor 設成 0 了
            double pidOutput = thetaPID.calculate(tx);

            // 把 PID 輸出當成比例係數，換成實際角速度
            // 方向如果反了，可以把這行改成 omega = +pidOutput * maxAngularRate;
            omega = -pidOutput * maxAngularRate;
        } else {
            // 沒目標就不要讓積分亂累，直接 reset
            thetaPID.reset();
        }

        // 4. 用 CTRE 的 setControl 寫法發送需求
        drivetrain.setControl(
            driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(omega)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // 放開按鈕 / command 結束時，要不要停下來看你習慣
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        thetaPID.reset();
    }

    @Override
    public boolean isFinished() {
        // 給 whileTrue 用，自己不會結束，靠按鈕放開
        return false;
    }
}