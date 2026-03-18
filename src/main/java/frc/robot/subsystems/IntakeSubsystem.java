package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeMotorChannelConstants;




public class IntakeSubsystem extends SubsystemBase{
     private Talon intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new Talon(IntakeMotorChannelConstants.k_intakeMotor);
    }

    public void onIntakeCommand() {
        intakeMotor.set(1);
    }

    public void offIntakeCommand() {
        intakeMotor.set(0);
    }
    


}
