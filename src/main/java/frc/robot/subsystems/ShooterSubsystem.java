package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    WPI_TalonSRX shooterMotorLeft = new WPI_TalonSRX(ShooterConstants.leftShooterPort);
    WPI_TalonSRX shooterMotorRight = new WPI_TalonSRX(ShooterConstants.rightShooterPort);
    WPI_TalonSRX pivotMotorLeft = new WPI_TalonSRX(ShooterConstants.leftPivotPort);
    WPI_TalonSRX pivotMotorRight = new WPI_TalonSRX(ShooterConstants.rightPivotPort);

    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(ShooterConstants.encoderPort);

    double desiredPivotAngle = 10;

    public ShooterSubsystem() {
        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true);

        pivotMotorLeft.setInverted(false);
        pivotMotorRight.setInverted(true);
    }

    public void setShooterSpeeds(double speed) {
        shooterMotorLeft.set(speed);
        shooterMotorRight.set(speed);
    }

    public void stopShooters() {
        setShooterSpeeds(0);
    }
}
