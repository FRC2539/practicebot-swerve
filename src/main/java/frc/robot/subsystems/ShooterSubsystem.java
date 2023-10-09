package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

public class ShooterSubsystem extends SubsystemBase {
    WPI_TalonSRX shooterMotorLeft = new WPI_TalonSRX(ShooterConstants.leftShooterPort);
    WPI_TalonSRX shooterMotorRight = new WPI_TalonSRX(ShooterConstants.rightShooterPort);
    WPI_TalonSRX pivotMotorLeft = new WPI_TalonSRX(ShooterConstants.leftPivotPort);
    WPI_TalonSRX pivotMotorRight = new WPI_TalonSRX(ShooterConstants.rightPivotPort);

    private IntakeMode intakeMode = IntakeMode.DISABLED;

    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(ShooterConstants.encoderPort);
    AnalogInput gamePieceSensor = new AnalogInput(ShooterConstants.shooterSensorPort);

    // values are currently extremely arbitrary
    PIDController pivotAngleController = new PIDController(3, 0, 2);

    double desiredPivotAngle = Math.PI / 2;

    public ShooterSubsystem() {
        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true);

        shooterMotorRight.follow(shooterMotorLeft);

        pivotMotorLeft.setInverted(false);
        pivotMotorRight.setInverted(false);

        pivotMotorRight.follow(pivotMotorLeft);
    }

    public Command pivotForward(double speed) {
        pivotMotorLeft.set(speed);
        pivotMotorRight.set(speed);
        return null;
    }

    public Command pivotBackward(double speed) {
        pivotMotorRight.set(speed*-1);
        pivotMotorRight.set(speed*-1);
        return null;
    }

    public void setShooterSpeeds(double speed) {
        shooterMotorLeft.set(speed);
    }

    public void stopShooters() {
        setShooterSpeeds(0);
    }

    public boolean hasGamePiece() {
        return gamePieceSensor.getValue() < 50;
    }   

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }

    public Command setDisabledCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.DISABLED);
                },
                () -> {});
    }

    public Command intakeModeCommand() {
        return startEnd(
                        () -> {
                            setIntakeMode(IntakeMode.INTAKE);
                        },
                        () -> {})
                .until(this::hasGamePiece);
    }

    public Command shootHighCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.HIGH);
                },
                () -> {});
    }

    public Command shootMidCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.MID);
                },
                () -> {});
    }

    public Command shootLowCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.LOW);
                },
                () -> {});
    }

    @Override
    public void periodic() {
        switch (intakeMode) {
            case DISABLED:
                desiredPivotAngle = Math.PI / 2;
                shooterMotorLeft.set(ControlMode.PercentOutput, 0);
                if(hasGamePiece()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.07);
                }
                break;
            case INTAKE:
                shooterMotorLeft.set(ControlMode.PercentOutput, 0.70);
                break;
            case HIGH:
                desiredPivotAngle = 1.2;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.90);
                break;
            case MID:
                desiredPivotAngle = 1;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.70);
                break;
            case LOW:
                desiredPivotAngle = 0;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.60);
                break;
        }
        pivotMotorLeft.set(pivotAngleController.calculate(pivotEncoder.getAbsolutePosition(), desiredPivotAngle));

    }

    public enum IntakeMode {
        DISABLED,
        INTAKE,
        HIGH,
        MID,
        LOW
    }
}
