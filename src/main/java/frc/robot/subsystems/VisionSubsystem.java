package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.vision.BackLimelight;
import frc.lib.vision.CameraInterfaces.Retroreflective;

public class VisionSubsystem extends SubsystemBase {
    private Retroreflective aprilTagCamera = new BackLimelight();

    public VisionSubsystem() {

    }

    public Retroreflective getLimelight() {
        return aprilTagCamera;
    }
}
