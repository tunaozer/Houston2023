// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

/*
 * TO DO:
 * getCurrentWheelSpeeds()
 * DriveConstants
 * Kinematic values
 * Characterization
 * PIDController values
 */

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
    private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

    private final RelativeEncoder m_frontLeftRelativeEncoder = m_frontLeft.getEncoder();
    private final RelativeEncoder m_rearLeftRelativeEncoder = m_rearLeft.getEncoder();
    private final RelativeEncoder m_frontRightRelativeEncoder = m_frontRight.getEncoder();
    private final RelativeEncoder m_rearRightRelativeEncoder = m_rearRight.getEncoder();

    private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    // sysid
    PIDController frontLeftPIDController = new PIDController(5.5938, 0, 0);
    PIDController rearLeftPIDController = new PIDController(5.5938, 0, 0);
    PIDController frontRightPIDController = new PIDController(5.5938, 0, 0);
    PIDController rearRightPIDController = new PIDController(5.5938, 0, 0);

    private final double kAutoaimTreshold = 0.25;


    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    PIDController rotPID = new PIDController(0.2, 0.05, 0);



    // The gyro sensor
    AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // Odometry class for tracking robot pose
    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
            DriveConstants.kDriveKinematics,
            m_gyro.getRotation2d(),
            new MecanumDriveWheelPositions());

    Pose2d pose;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders
        
  /*       m_frontLeftRelativeEncoder.setDistancePerPulse()
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    */
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_frontRight.setInverted(true);
        m_rearRight.setInverted(true);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());

        pose = m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());

        SmartDashboard.putNumber("frontLeftEncoder", m_frontLeftRelativeEncoder.getPosition());
        SmartDashboard.putNumber("rearLeftEncoder", m_rearLeftRelativeEncoder.getPosition());
        SmartDashboard.putNumber("frontRightEncoder", m_frontRightRelativeEncoder.getPosition());
        SmartDashboard.putNumber("rearRightEncoder", m_rearRightRelativeEncoder.getPosition());

        SmartDashboard.putNumber("gyroRotations", m_gyro.getRotation2d().getRotations());
        SmartDashboard.putNumber("gyroDegrees", m_gyro.getRotation2d().getDegrees());
        SmartDashboard.putNumber("gyroRadians", m_gyro.getRotation2d().getRadians());

        SmartDashboard.putNumber("velocity_frontLeftEncoder", m_frontLeftRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("velocity_rearLeftEncoder", m_rearLeftRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("velocity_frontRightEncoder", m_frontRightRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("velocity_rearRightEncoder", m_rearRightRelativeEncoder.getVelocity());

        SmartDashboard.putNumber("reflector-error", tx.getDouble(0));
        SmartDashboard.putNumber("rotPIDcalculate", rotPID.calculate(tx.getDouble(0), 0));


    }

    
    public Pose2d getPose() {
        return pose;
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
    }

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
        } else {
            m_drive.driveCartesian(xSpeed, ySpeed, rot);
        }
    }

    public void resetEncoders() {
        m_frontLeftRelativeEncoder.setPosition(0);
        m_rearLeftRelativeEncoder.setPosition(0);
        m_frontRightRelativeEncoder.setPosition(0);
        m_rearRightRelativeEncoder.setPosition(0);
      }
    
    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        m_frontLeft.setVoltage(volts.frontLeftVoltage / 12);
        m_rearLeft.setVoltage(volts.rearLeftVoltage / 12);
        m_frontRight.setVoltage(volts.frontRightVoltage / 12);
        m_rearRight.setVoltage(volts.rearRightVoltage / 12);
    }
    
    public RelativeEncoder getFrontLeftEncoder() {
        return m_frontLeftRelativeEncoder;
    }

    
    public RelativeEncoder getRearLeftEncoder() {
        return m_rearLeftRelativeEncoder;
    }

    
    public RelativeEncoder getFrontRightEncoder() {
        return m_frontRightRelativeEncoder;
    }

    
    public RelativeEncoder getRearRightEncoder() {
        return m_rearRightRelativeEncoder;
    }

    public PIDController getFrontLeftPIDController() {
        return frontLeftPIDController;
    }
    public PIDController getRearLeftPIDController() {
        return rearLeftPIDController;
    }
    public PIDController getFrontRightPIDController() {
        return frontRightPIDController;
    }
    public PIDController getRearRightPIDController() {
        return rearRightPIDController;
    }

    // RPM of the motor -> RPM of the wheel -> METERS per minute ->  METERS per second
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                // .getVelocity() / gearRatio * 2 * Math.PI * wheelRadius / 60
                m_frontLeftRelativeEncoder.getVelocity(),
                m_rearLeftRelativeEncoder.getVelocity(),
                m_frontRightRelativeEncoder.getVelocity(),
                m_rearRightRelativeEncoder.getVelocity());
    }
    /**
     * Gets the current wheel distance measurements.
     *
     * @return the current wheel distance measurements in a
     *         MecanumDriveWheelPositions object.
     */
    public MecanumDriveWheelPositions getCurrentWheelDistances() {
        return new MecanumDriveWheelPositions(
                m_frontLeftRelativeEncoder.getPosition(),
                m_rearLeftRelativeEncoder.getPosition(),
                m_frontRightRelativeEncoder.getPosition(),
                m_rearRightRelativeEncoder.getPosition());
    }

    //limelight
    public void autoaimRobot() {
        double rotPidout = -rotPID.calculate(tx.getDouble(0), 0) / 60;

        if (tx.getDouble(0) < -7)
        {
            // left
            drive(0, -0.1, 0, true);
        }
        else if (tx.getDouble(0) > -4)
        {
            // right
            drive(0, 0.1, 0, true);
        }
        else
        {
            drive(0, 0, 0, true);
        }

    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }
    

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public PIDController getFrontLeftPidController() {
        return frontLeftPIDController;
    }

    public PIDController getRearLeftPidController() {
        return rearLeftPIDController;
    }

    public PIDController getFrontRightPidController() {
        return frontRightPIDController;
    }

    public PIDController getRearRightPidController() {
        return rearRightPIDController;
    }
}
