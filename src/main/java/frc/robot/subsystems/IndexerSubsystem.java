package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.IndexMotorChannelConstants;



public class IndexerSubsystem {
     private Talon indexMotor;

    public IndexerSubsystem() {
        indexMotor = new Talon(IndexMotorChannelConstants.k_indexMotor);
    }

    public void spinShooterCommand() {
        indexMotor.set(1);
    }

    public void stopShooterCommand() {
        indexMotor.set(0);
    }
    


}
