package frc.robot;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }

    public static final class ClimbConstants {
        public static final int climb1Port=6;
        public static final int climb2Port=8;

        public static final double kP=0.05;
        public static final double kI=0;
        public static final double kD=0.005;

        // measured in relative encoder measurement
        public static final double[] kSetPoints = {0, 1050, 1450};
        public static final double kPickUpSetPoint = 1475;

        public static final double kMaxClimberSpeed = 0.25;

        public static final double kClimbPIDFactor = 70;
    }
    public static final class IntakeConstants {
        public static final int IntakePort=5;
        public static final int RollerPort=7;
    }
    public static final class AutoConstants {
        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.5;

        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kMaxAccelerationMetersPerSecondSquared=3;

        public static final TrapezoidProfile.Constraints kPThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final double kMaxSpeedMetersPerSecond = 0.5;
    }
    public static final class DriveConstants {
        public static final int kFrontLeftMotorPort = 15;
        public static final int kRearLeftMotorPort = 16;
        public static final int kFrontRightMotorPort = 14;
        public static final int kRearRightMotorPort = 9;

        public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRearLeftEncoderPorts = new int[] {2, 3};
        public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
        public static final int[] kRearRightEncoderPorts = new int[] {6, 7};
        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.37721, 3.7906, 1.1659);
        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kRearLeftEncoderReversed = true;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kRearRightEncoderReversed = true;

        public static final double kTrackWidth = 56;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 52;
        // Distance between centers of front and back wheels on robot

        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        /*
        public static final SimpleMotorFeedforward kFeedforward =
            new SimpleMotorFeedforward(0.37721, 3.7906, 1.1659); // moved to drivesubsystem;
        */
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPFrontLeftVel = 0.5;
        public static final double kPRearLeftVel = 0.5;
        public static final double kPFrontRightVel = 0.5;
        public static final double kPRearRightVel = 0.5;
    }
}