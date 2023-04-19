// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClimbStageCommand;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSub. */
  CANSparkMax climb1;
  CANSparkMax climb2;

  public RelativeEncoder climb1encoder;
  public RelativeEncoder climb2encoder;

  public int climbCurrentStage;

  public PIDController m_PidController = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kP);

  public boolean isPickupStageMode = false;
  public boolean isManualMode = false;

  double manualSpeed;

  public ClimbSubsystem() {
    climb1 = new CANSparkMax(ClimbConstants.climb1Port, MotorType.kBrushless);
    climb2 = new CANSparkMax(ClimbConstants.climb2Port, MotorType.kBrushless);
    climb2.follow(climb1);

    climb1encoder = climb1.getEncoder();
    climb2encoder = climb2.getEncoder();
  }

  // encoder 1:
  // stage 2 = 1425
  // stage 1 = 990
  // stage 0 = 5

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FOLLOW_Can1Pos", climb1encoder.getPosition());
    SmartDashboard.putNumber("MAIN_Can2Pos", climb2encoder.getPosition());
    SmartDashboard.putNumber("PID_Calculate", m_PidController.calculate(climb1encoder.getPosition()));
    SmartDashboard.putNumber("Climbcurrentstage", climbCurrentStage);

    double finalSpeed = climb1encoder.getPosition();

    // PID / 70 = 1
    double finalPID = m_PidController.calculate(climb1encoder.getPosition()) / ClimbConstants.kClimbPIDFactor;

    SmartDashboard.putNumber("finalPID_before", finalPID);

    // < 0.05 finalpid =

    if (!isManualMode) {
      // Clamp finalPID betweeb -1/+1
      if (finalPID > 1) {
        finalPID = 1;
      } else if (finalPID < -1) {
        finalPID = -1;
      }

      if (finalPID > 0.04) {
        finalPID = 0.25;
      } else if (finalPID <= 0.04 && finalPID >= 0) {
        finalPID = m_PidController.calculate(climb1encoder.getPosition()) / ClimbConstants.kClimbPIDFactor;
      } else if (finalPID < 0) {
        finalPID = -0.2;
      }

      // stopswitch command
      if (climbCurrentStage == 0 && climb1encoder.getPosition() < 20 && !isPickupStageMode) {
        finalPID = 0;
        finalPID = 0;

        climb1encoder.setPosition(0);
        climb2encoder.setPosition(0);
      }

      climb1.set(finalPID);
      climb2.set(finalPID);
    }

    SmartDashboard.putNumber("finalPID_final", finalPID);
    SmartDashboard.putNumber("PID_SetPosition", m_PidController.getSetpoint());
    SmartDashboard.putNumber("finalSpeed", finalSpeed);
    SmartDashboard.putBoolean("isAutoStageMode", isPickupStageMode);
    SmartDashboard.putNumber("manualSpeed", manualSpeed);
  }

  public void climbMove(int index) {

    if (!isPickupStageMode) {
      // set setpoint to current index = stage
      m_PidController.setSetpoint(ClimbConstants.kSetPoints[index]);
    } else {
      // set setpoint to pickup index
      m_PidController.setSetpoint(ClimbConstants.kPickUpSetPoint);
    }
  }

  public void encoderInit() {
    climbCurrentStage = 0;
    m_PidController.reset();
    climbMove(climbCurrentStage);

    // new test
    climb1encoder.setPosition(0);
    climb2encoder.setPosition(0);
  }

  public void setAutoStageMode() {
    isPickupStageMode = true;

    isManualMode = false;

    m_PidController.setSetpoint(ClimbConstants.kPickUpSetPoint);
  }

  public void resetEncoders(){
    climb1encoder.setPosition(0);
    climb2encoder.setPosition(0);

    climbCurrentStage=0;
  }

  public void teleopClimbUp() {
    climb1.set(0.5);
    climb2.set(0.5);

    manualSpeed = 0.5;
  }

  public void teleopClimbDown() {
    climb1.set(-0.2);
    climb2.set(-0.2);

    manualSpeed = -0.2;
  }

  public void teleopClimbStop() {
    climb1.set(0.03);
    climb2.set(0.03);

    manualSpeed = 0.03;
  }

}
