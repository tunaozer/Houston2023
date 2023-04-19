// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbStageCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.RollerSubystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.event.EventLoop;





/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  public final ClimbSubsystem m_climb =new ClimbSubsystem();
  private final IntakeSubsystem m_intake= new IntakeSubsystem();
  private final RollerSubsystem m_roller =new RollerSubsystem();
  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_drive.drive(
                    //-m_driverController.getLeftY(),
                    //-m_driverController.getRightX(),
                                                                                                                                                                       //-m_driverController.getLeftX(),
                    -m_driverController.getLeftY(),
                    m_driverController.getLeftX(),
                    m_driverController.getRawAxis(2),                                    
                    false),
            m_drive));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, 10)
        .onTrue(new InstantCommand(() -> m_drive.setMaxOutput(0.2)))
        .onFalse(new InstantCommand(() -> m_drive.setMaxOutput(1)));


        new JoystickButton(m_driverController,9)
        .whileTrue(new RunCommand(() -> m_drive.autoaimRobot()));

        // autostage
        new JoystickButton(m_driverController, 5)
                .onTrue(new ClimbStageCommand(m_climb, false, false, 0));
        new JoystickButton(m_driverController, 6)
                .onTrue(new ClimbStageCommand(m_climb, true, false, 0));
        // pickup stage
        new JoystickButton(m_driverController, 1)
                .onTrue(new InstantCommand(() -> m_climb.setAutoStageMode()));
        // manual
        new POVButton(m_driverController,0)
                .whileTrue(new ClimbStageCommand(m_climb, false, true, 1))
                .whileFalse(new ClimbStageCommand(m_climb, false, true, 0));
        new POVButton(m_driverController,180 )
                .whileTrue(new ClimbStageCommand(m_climb, true, true, 2))
                .whileFalse(new ClimbStageCommand(m_climb, true, true, 0));

        // indexer open - close
        new JoystickButton(m_driverController, 4)
                .onTrue(new InstantCommand(() -> m_intake.change_Pneumatic_Intake()))
                .onFalse(new InstantCommand(() -> m_intake.IntakeStop()));

        // intake roll - unroll
        new JoystickButton(m_driverController, 2)
                .onTrue(new InstantCommand(() -> m_roller.IntakeRoll()))
                .onFalse(new InstantCommand(() -> m_roller.IntakeStop()));
        new JoystickButton(m_driverController, 3)
                .onTrue(new InstantCommand(() -> m_roller.IntakeUnroll()))
                .onFalse(new InstantCommand(() -> m_roller.IntakeStop()));

        new  POVButton(m_driverController,90)
                .onTrue(new InstantCommand(()->m_climb.resetEncoders()));    
    }






  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /* *//* */ 
public Command getAutonomousCommand(){


    // Run path following command, then stop at the end. 
    return new SequentialCommandGroup(
        new SequentialCommandGroup(
        new RunCommand(()->m_climb.climbMove(2),m_climb)).withTimeout(1.8)
    .andThen(new InstantCommand(()-> m_climb.teleopClimbStop(),m_climb))
    .andThen(new RunCommand(()->m_intake.IntakeOpen(),m_climb)).withTimeout(1)
    .andThen(new RunCommand(()->m_roller.IntakeUnroll(),m_roller)).withTimeout(1.5)
    .andThen(new RunCommand(()->m_climb.climbMove(0),m_climb)).withTimeout(1).andThen(new WaitCommand(1))
    .andThen(new RunCommand(()->m_drive.drive(-0.4,0,0,false),m_drive)).withTimeout(2.5)
    .andThen(() -> m_drive.drive(0, 0, 0, false)));
    
    //return new RunCommand(m_climb::climbActive, m_climb).withTimeout(1.5).andThen(new InstantCommand(m_climb::climbStop,m_climb)).andThen(new InstantCommand(m_intake::IntakeUp,m_intake).withTimeout(0.5).andThen(new InstantCommand(m_intake2::IntakeRoll,m_intake2)).withTimeout(2).andThen(m_intake2::IntakeStop,m_intake2).andThen(mecanumControllerCommand2).andThen(() -> m_robotDrive.drive(0, 0, 0, false)));
  /* 
    return new RunCommand(m_climb::climbActive, m_climb).withTimeout(1.5)
    .andThen(new InstantCommand(m_climb::climbStop,m_climb))
    .andThen(new RunCommand(m_intake::IntakeUp,m_intake).withTimeout(1.5)
    .andThen(new RunCommand(m_intake2::IntakeRoll,m_intake2)).withTimeout(3)
    .andThen(new InstantCommand(m_intake::IntakeStopke,m_intake))
    .andThen(new InstantCommand(m_intake2::IntakeStop,m_intake2))
    .andThen(new RunCommand(m_climb::climbDisable,m_climb)).withTimeout(1)
    .andThen(mecanumControllerCommand)
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false)));
  */
}
  }
