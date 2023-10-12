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
    PIDController pivotAngleController = new PIDController(5, 0, 0);

    double desiredPivotAngle;

    public ShooterSubsystem() {
        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true);

        shooterMotorRight.follow(shooterMotorLeft);

        pivotMotorLeft.setInverted(true);
        pivotMotorRight.setInverted(true);

        pivotMotorRight.follow(pivotMotorLeft);

        pivotAngleController.enableContinuousInput(0, 1);
        pivotAngleController.setTolerance(0.025);
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
    
    public void bringIntakeUpright() {
        desiredPivotAngle = 0.25;
        double outputUsed = pivotAngleController.calculate(-(pivotEncoder.getAbsolutePosition() - 0.0497), desiredPivotAngle);
        pivotMotorLeft.set(outputUsed);
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
                        () -> {});
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
                desiredPivotAngle = -.947 + 1.0 / 6;
                if(hasGamePiece()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.07);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0);
                }
                break;
            case INTAKE:
                desiredPivotAngle = -.947 - 0.027777778;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.70);
                break;
            case HIGH:
                desiredPivotAngle = -.947 + 1.0 / 5;
                if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.80);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.07);
                }
                break;
            case MID:
                desiredPivotAngle = -.947 + 1.0 / 9;
                if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.90);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.07);
                }
                break;
            case LOW:
                desiredPivotAngle = -.947;
                if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.60);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.07);
                }
                break;
        }
        double outputUsed = pivotAngleController.calculate(-(pivotEncoder.getAbsolutePosition() - 0.0497), desiredPivotAngle);
        pivotMotorLeft.set(outputUsed);
        // pivotMotorLeft.set(desiredPivotAngle);
        System.out.println(outputUsed);

    }

    public enum IntakeMode {
        DISABLED,
        INTAKE,
        HIGH,
        MID,
        LOW
    }
}
