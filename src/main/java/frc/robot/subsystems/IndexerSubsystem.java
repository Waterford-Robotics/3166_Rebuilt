package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexMotorChannelConstants;



public class IndexerSubsystem extends SubsystemBase{
     private Talon indexMotor;

    public IndexerSubsystem() {
        indexMotor = new Talon(IndexMotorChannelConstants.k_indexMotor);
    }

    public void spinIndexerCommand() {
        indexMotor.set(1);
    }

    public void stopIndexerCommand() {
        indexMotor.set(0);
    }
    


}
