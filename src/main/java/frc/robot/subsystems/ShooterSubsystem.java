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

    int i = 0;

    public ShooterSubsystem() {
        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true);

        shooterMotorRight.follow(shooterMotorLeft);

        pivotMotorLeft.setInverted(true);
        pivotMotorRight.setInverted(true);

        pivotMotorRight.follow(pivotMotorLeft);

        pivotAngleController.enableContinuousInput(0, 1);
        pivotAngleController.setTolerance(0.035);
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

    public Command setUprightCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.STRAIGHT_UP);
                },
                () -> {});
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

    public Command shootChargingStation() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.CHARGINGSTATION);
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
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.14);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.14);
                }
                break;
            case INTAKE:
                desiredPivotAngle = -.947 - 0.047777778;//0.027777778
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.70);
                break;
            case CHARGINGSTATION:
                desiredPivotAngle = -.947 + 1.0 / 9; //across charging station
                if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 1); //across charging station
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.14);
                }
                break;
            case HIGH:
                desiredPivotAngle = -.947 + 1.0 / 5;
                if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.80);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.14);
                }
                break;
            case MID:

                desiredPivotAngle = -.947 + 1.0 / 5; //mid
                if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.45); //mid
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, -0.14);
                }
                break;
            case LOW:
                desiredPivotAngle = -.947;
                //if(pivotAngleController.atSetpoint()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.35);
                //} else {
                //    shooterMotorLeft.set(ControlMode.PercentOutput, -0.14);
                //}
                break;
            case STRAIGHT_UP:
                desiredPivotAngle = .25 + -.947;
                shooterMotorLeft.set(ControlMode.PercentOutput, 0.0);
                break;
        }
        double outputUsed = pivotAngleController.calculate(-(pivotEncoder.getAbsolutePosition() - 0.715), desiredPivotAngle);
        if (outputUsed > 0) {
            outputUsed += .10;
        } else {
            outputUsed -= .10;
        }
        pivotMotorLeft.set(outputUsed);
        // pivotMotorLeft.set(desiredPivotAngle);
        if (i == 0) {
            System.out.println(pivotEncoder.getAbsolutePosition());
        }
        i++;
        i %= 20;
    }

    public enum IntakeMode {
        DISABLED,
        INTAKE,
        CHARGINGSTATION,
        HIGH,
        MID,
        LOW,
        STRAIGHT_UP
    }
}