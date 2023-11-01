package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    WPI_TalonSRX shooterMotorLeft = new WPI_TalonSRX(ShooterConstants.leftShooterPort);
    WPI_TalonSRX shooterMotorRight = new WPI_TalonSRX(ShooterConstants.rightShooterPort);
    WPI_TalonSRX pivotMotorLeft = new WPI_TalonSRX(ShooterConstants.leftPivotPort);
    WPI_TalonSRX pivotMotorRight = new WPI_TalonSRX(ShooterConstants.rightPivotPort);

    private IntakeMode intakeMode = IntakeMode.DISABLED;

    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(ShooterConstants.encoderPort);
    AnalogInput gamePieceSensor = new AnalogInput(ShooterConstants.shooterSensorPort);

    // values are currently extremely arbitrary
    PIDController pivotAngleController = new PIDController(3, 0, 0);

    double desiredPivotAngle;
    double desiredShooterSpeed;

    public Optional<Double> speedOverride = Optional.empty();

    int i = 0;

    public ShooterSubsystem() {
        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true);

        shooterMotorRight.follow(shooterMotorLeft);

        pivotMotorLeft.setInverted(true);
        pivotMotorRight.setInverted(true);

        pivotMotorRight.follow(pivotMotorLeft);

        pivotAngleController.enableContinuousInput(0, 1);
        pivotAngleController.setTolerance(0.015);
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
                // desiredPivotAngle = -.947 + 1.0 / 6;
                desiredPivotAngle = .24 + -.947;
                if (hasGamePiece()) {
                    desiredShooterSpeed =  -0.14;
                } else {
                    desiredShooterSpeed =  -0.14;
                }
                break;
            case INTAKE:
                desiredPivotAngle = -.947  - (0.047777778 / 2); // 0.027777778
                desiredShooterSpeed =  -0.70;
                break;
            case CHARGINGSTATION:
                desiredPivotAngle = -.947 + 1.0 / 9; // across charging station
                if (pivotAngleController.atSetpoint()) {
                    desiredShooterSpeed =  1; // across charging station
                } else {
                    desiredShooterSpeed =  -0.14;
                }
                break;
            case HIGH:
                desiredPivotAngle = -.947 + (1.0 / 6);
                if (pivotAngleController.atSetpoint()) {
                    desiredShooterSpeed =  0.80;
                } else {
                    desiredShooterSpeed =  -0.14;
                }
                break;
            case MID:
                desiredPivotAngle = -.947 + 1.0 / 6; // mid
                if (pivotAngleController.atSetpoint()) {
                    desiredShooterSpeed =  0.45; // mid
                } else {
                    desiredShooterSpeed =  -0.14;
                }
                break;
            case LOW:
                desiredPivotAngle = -.947;
                if(Math.abs(pivotAngleController.getPositionError()) < 1.0 / 8) {
                    desiredShooterSpeed =  0.30;
                } else {
                   desiredShooterSpeed =  -0.14;
                }
                break;
            case STRAIGHT_UP:
                desiredPivotAngle = .25 + -.947;
                desiredShooterSpeed =  0.0;
                break;
        }
        double outputUsed =
                pivotAngleController.calculate(-(pivotEncoder.getAbsolutePosition() - 0.715), desiredPivotAngle);
        if (outputUsed > 0) {
            outputUsed += .10;
        } else {
            outputUsed -= .10;
        }
        pivotMotorLeft.set(outputUsed);

        shooterMotorLeft.set(ControlMode.PercentOutput, speedOverride.orElse(desiredShooterSpeed));
        // pivotMotorLeft.set(desiredPivotAngle);
        if (i == 0) {
            //System.out.println(pivotEncoder.getAbsolutePosition());
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
