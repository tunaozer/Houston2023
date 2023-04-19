package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbStageCommand extends CommandBase {

    private final ClimbSubsystem m_climb;

    private boolean isIncrease;
    private boolean isManualMode;

    private double manualInputType;

    private boolean commandIsFinished = false;

    public ClimbStageCommand(ClimbSubsystem m_climb, boolean isIncrease, boolean isManualMode, double manualInputType) {
        this.m_climb = m_climb;
        this.isIncrease = isIncrease;
        this.isManualMode = isManualMode;
        this.manualInputType = manualInputType;
        addRequirements(m_climb);
    }

    @Override
    public void initialize() {

        m_climb.isManualMode = isManualMode;

        if (!isManualMode)
        {
            if (isIncrease && m_climb.climbCurrentStage + 1 < ClimbConstants.kSetPoints.length) {
                if (!m_climb.isPickupStageMode)
                {
                    m_climb.climbCurrentStage = m_climb.climbCurrentStage + 1;
                }
            } else if (!isIncrease && m_climb.climbCurrentStage - 1 > -1) {
                // decrease stage
                if (!m_climb.isPickupStageMode)
                {
                    m_climb.climbCurrentStage = m_climb.climbCurrentStage - 1;
                }
            }
    
            if (m_climb.isPickupStageMode)
            {
                m_climb.isPickupStageMode = false;
            }
    
            m_climb.climbMove(m_climb.climbCurrentStage);
    
            commandIsFinished = true;
        }
        else 
        {
            if (manualInputType == 2)
            {
                m_climb.teleopClimbUp();
            }
            else if (manualInputType == 1)
            {
                m_climb.teleopClimbDown();
            }
            else if (manualInputType == 0)
            {
                m_climb.teleopClimbStop();
            }
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (commandIsFinished) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

}
