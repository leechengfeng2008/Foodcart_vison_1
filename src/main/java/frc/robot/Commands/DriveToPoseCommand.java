package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToPoseCommand extends Command {
    private final Swerve m_swerve;
    private final Pose2d m_targetPose;

    // 設定 PID 參數與限制 (最大速度 3m/s, 最大加速度 3m/s^2)
    private final ProfiledPIDController xController = new ProfiledPIDController(0.3, 0, 0, 
        new TrapezoidProfile.Constraints(0.3, 3.0));
    private final ProfiledPIDController yController = new ProfiledPIDController(0.3, 0, 0, 
        new TrapezoidProfile.Constraints(0.3, 3.0));
    // 角度 PID (最大轉速 2pi/s)
    private final ProfiledPIDController thetaController = new ProfiledPIDController(4.0, 0, 0, 
        new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2));

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    /**
     * @param swerve 傳入底盤 subsystem
     * @param x 目標 X 座標 (米)
     * @param y 目標 Y 座標 (米)
     * @param headingDegrees 目標朝向角度 (度)
     */
    public DriveToPoseCommand(Swerve swerve, double x, double y, double headingDegrees) {
        this.m_swerve = swerve;
        this.m_targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
        
        addRequirements(m_swerve);

        // 設置容許誤差 (例如 5公分, 2度)
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Math.toRadians(2.0));
        
        // 角度循環補償
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        var currentPose = m_swerve.getState().Pose;
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        var currentPose = m_swerve.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX(), m_targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), m_targetPose.getY());
        double thetaSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(), 
            m_targetPose.getRotation().getRadians()
        );

        m_swerve.setControl(
            m_driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(thetaSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.setControl(new SwerveRequest.Idle());
    }
}