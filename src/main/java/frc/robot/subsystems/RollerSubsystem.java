/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
    WPI_TalonSRX intake;

    public RollerSubsystem() {
        intake = new WPI_TalonSRX(Constants.IntakeConstants.RollerPort);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // 0.4 / -0.4
    public void IntakeRoll() {
        intake.set(ControlMode.PercentOutput, 0.8);

    }

    public void IntakeUnroll() {
        intake.set(ControlMode.PercentOutput, -0.6);

    }

    public void IntakeStop() {
        intake.set(ControlMode.PercentOutput, 0);
    }
}
