package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.library.until.PolynomialRegression;

public class Constants {
    

    public static final class PhotonAprilTagConstants{

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    public static final PolynomialRegression xyStdDevModel =
        new PolynomialRegression(
            new double[] {
                0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                3.223358, 4.093358, 4.726358
            },
            new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
            2);
    public static final PolynomialRegression thetaStdDevModel =
            new PolynomialRegression(
                new double[] {
                    0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                    3.223358, 4.093358, 4.726358
                },
                new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
            1);
    }

    public static final class VisionConstants {
            // TODO : Need to be Tuned.
            // public static final PIDConfig VISION_FEEDBACK = new PIDConfig(0, 0, 0);
            // public static final SimpleMotorFeedforward VISION_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);

            public static final PIDController ALGAE_VISION_FEEDBACK = new PIDController(0, 0, 0);
            public static final double ALGAE_VISION_AIM_TOLERANCE = 0;
            public static final double ALGAE_VISION_AIM_INTEGRATOR_RANGE = 0.5;

            public static final double MIN_AREA_FOR_MEGATAG = 0.4; //TODO: test values
    
            public static final double frontLLToRobotX = 0.0; //TODO: find values
            public static final double frontLLToRobotY = 0.0; //TODO: find values\
            public static final double frontLLToRobotZ = 0.0; //TODO: find values
            public static final double frontLLroll = 0.0; //TODO: find values
            public static final double frontLLpitch = 0.0; //TODO: find values
            public static final double frontLLyaw = 0.0; //TODO: find values
            public static final Rotation2d frontLLToRobotTheta = new Rotation2d();  //TODO: find values

            public static final double backLLToRobotX = 0.0; //TODO: find values
            public static final double backLLToRobotY = 0.0; //TODO: find values
            public static final double backLLToRobotZ = 0.0; //TODO: find values
            public static final double backLLroll = 0.0; //TODO: find values
            public static final double backLLpitch = 0.0; //TODO: find values
            public static final double backLLyaw = 0.0; //TODO: find values
            public static final Rotation2d backLLToRobotTheta = new Rotation2d(0.0); //TODO: find values

            public static final double upperLLToRobotX = 0.0; //TODO: find values
            public static final double upperLLToRobotY = 0.0; //TODO: find values
            public static final double upperLLToRobotZ = 0.0; //TODO: find values
            public static final double upperLLroll = 0.0; //TODO: find values
            public static final double upperLLpitch = 0.0; //TODO: find values
            public static final double upperLLyaw = 0.0; //TODO: find values
            public static final Rotation2d upperLLToRobotTheta = new Rotation2d(); //TODO: find values

            public static final Matrix<N3,N1> odometryStdDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)); //TODO: find values 
            public static final Matrix<N3,N1> visionStdDev = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(1.0)); //TODO: find values

            //Todo: get PID VALUES 
            public enum limelightStrafePID{
                KP(0.6),
                KI(0.0),
                KD(0.0);

                private final double value;

                private limelightStrafePID(double value){
                    this.value = value;
                }

                public double getValue(){
                    return value;
                }
            }


            
            public enum limelightRotationPID{
                KP(0.5),
                KI(0.0),
                KD(0.0);

                private final double value;

                private limelightRotationPID(double value){
                    this.value = value;
                }

                public double getValue(){
                    return value;
                }
            }


            
            public enum limelightTranslatePID{
                KP(0.3),
                KI(0.0),
                KD(0.0);

                private final double value;

                private limelightTranslatePID(double value){
                    this.value = value;
                }

                public double getValue(){
                    return value;
                }
            }
        }
}
