package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexMotorChannelConstants;
import frc.robot.Constants.MotorChannelConstants;
import frc.robot.Constants.IndexerConstants;


public class IndexerSubsystem extends SubsystemBase{
    private Talon IndexMotor;
    private Talon ShooterintakeMotor;
    public IndexerSubsystem() {
        IndexMotor = new Talon(IndexMotorChannelConstants.k_indexMotor);
        ShooterintakeMotor = new Talon(MotorChannelConstants.k_shooterIntake);

    }

    public void startIndexerCommand() {
        IndexMotor.set(IndexerConstants.k_IndexerSpeed);
        ShooterintakeMotor.set(IndexerConstants.k_ShooterIntakeSpeed);
    }

    public void stopIndexerCommand() {
        IndexMotor.set(0);
        ShooterintakeMotor.set(0);
    }
    


}
