package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
    PWMSparkMax blinkinPWMController = new PWMSparkMax(0);
    
    public LightsSubsystem() {

    }

    public void setLEDS(double ledValue) {
        blinkinPWMController.set(ledValue);
    }
}
