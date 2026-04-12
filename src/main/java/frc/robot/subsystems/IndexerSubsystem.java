package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexCIDConstants;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterCIDConstants;


public class IndexerSubsystem extends SubsystemBase{
    private TalonFX IndexMotor;
    private TalonFX ShooterintakeMotor;
    public IndexerSubsystem() {
        IndexMotor = new Talon(IndexCIDConstants.k_indexMotor);
        ShooterintakeMotor = new Talon(ShooterCIDConstants.k_shooterIntake);

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
