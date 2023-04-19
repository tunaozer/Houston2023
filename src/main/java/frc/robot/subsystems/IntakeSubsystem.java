/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new ClimbSub. */
    WPI_TalonSRX intake;

    private boolean intake_pneumatic_state = false;
    // private boolean intake_motor_state = false;

    public IntakeSubsystem() {
        intake = new WPI_TalonSRX(Constants.IntakeConstants.IntakePort);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IntakePneumaticState", intake_pneumatic_state);
    }

    public void change_Pneumatic_Intake() {
        if (intake_pneumatic_state) {
            IntakeOpen();
        } 
        else
        {
            IntakeClose();
        }   
    }

    public void IntakeOpen() {
        intake.set(ControlMode.PercentOutput, 0.3);
        intake_pneumatic_state = false;
    }

    public void IntakeClose() {
        intake.set(ControlMode.PercentOutput, -0.3);
        intake_pneumatic_state = true;
    }

    public void IntakeStop() {
        intake.set(ControlMode.PercentOutput, 0);
    }

}
