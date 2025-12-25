// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AlignCommand;
import frc.robot.Commands.CloseTagCommand;
import frc.robot.Commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;




public class RobotContainer {

  private final CommandXboxController controller_1 = new CommandXboxController(0);
  private final CommandXboxController controller_2 = new CommandXboxController(1);
  private final CommandXboxController controller_3 = new CommandXboxController(2);
  public final  Swerve drivetrain = TunerConstants.createDrivetrain();
  private final Consumer<VisionFieldPoseEstimate> visionFieldPoseEstimateConsumer = new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
            drivetrain.addVisionMeasurement(visionFieldPoseEstimate);
            // SmartDashboard.putNumber("vision X into drivetrain", visionFieldPoseEstimate.getVisionRobotPoseMeters().getX());
            // SmartDashboard.putNumber("vision Y into drivetrain", visionFieldPoseEstimate.getVisionRobotPoseMeters().getY());
        }
    };
  public final VisionState visionState = new VisionState(visionFieldPoseEstimateConsumer);
  public final VisionSubsystem visionSubsystem = new VisionSubsystem(visionState, drivetrain);
  public final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);

  // 最大線速度（公尺/秒）
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  // 最大角速度（弧度/秒）
  private double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond); 
  private double MaxAngularRate1 = RotationsPerSecond.of(10).in(RadiansPerSecond); 



  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        // 線速度和角速度都加上 10% 的死區
        .withDeadband(1)
        .withRotationalDeadband(MaxAngularRate * 0.1) 
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  // 剎車模式
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // 輪朝某個方向指過去的請求
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // 以「機器人自身座標(Robot Centric)」控制，直線前進用的驅動請求
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Telemetry：記錄/回報底盤資訊的 logger，建構時傳入最大速度
  private final Telemetry logger = new Telemetry(MaxSpeed);


  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
    // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() ->
      drive.withVelocityX(-controller_1.getLeftY()* MaxSpeed) // Drive forward with negative Y (forward)
      .withVelocityY(-controller_1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(-controller_1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
      final var idle = new SwerveRequest.Idle();
      RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    
        controller_1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller_1.b().whileTrue(drivetrain.applyRequest(
          () ->point.withModuleDirection(new Rotation2d(-controller_1.getLeftY(), -controller_1.getLeftX()))));
        controller_1.x().whileTrue(drivetrain.applyRequest(
          ()->point.withModuleDirection(Rotation2d.fromDegrees(0))));
        controller_1.pov(0).whileTrue(drivetrain.applyRequest(()->
          forwardStraight.withVelocityX(MaxSpeed).withVelocityY(0)));
        controller_1.pov(180).whileTrue(drivetrain.applyRequest(()->
          forwardStraight.withVelocityX(-MaxSpeed).withVelocityY(0)));

        controller_1.leftBumper().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric()));
        controller_1.rightBumper().whileTrue(new AlignCommand(drivetrain, visionSubsystem, controller_1, MaxSpeed, MaxAngularRate1));
        controller_1.leftTrigger().whileTrue(new CloseTagCommand(drivetrain, visionSubsystem, controller_1, MaxSpeed));
        controller_1.y().whileTrue(new DriveToPoseCommand(drivetrain, 0.01,0.01, 0));



        controller_2.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller_2.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller_2.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller_2.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // controller_3.a().whileTrue((new AlignCommand(drivetrain, visionSubsystem, controller_1, MaxSpeed, MaxAngularRate1)));


        drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
