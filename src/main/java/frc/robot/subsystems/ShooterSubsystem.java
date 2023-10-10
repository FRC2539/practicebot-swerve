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
    PIDController pivotAngleController = new PIDController(20, 0, 0);

    double desiredPivotAngle = Math.PI / 2;

    public ShooterSubsystem() {
        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true);

        shooterMotorRight.follow(shooterMotorLeft);

        pivotMotorLeft.setInverted(true);
        pivotMotorRight.setInverted(true);

        pivotMotorRight.follow(pivotMotorLeft);

        pivotAngleController.enableContinuousInput(0, 1);
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
                desiredPivotAngle = -.947 + 1.0/4;
                if(hasGamePiece()) {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0.07);
                } else {
                    shooterMotorLeft.set(ControlMode.PercentOutput, 0);
                }
                break;
            case INTAKE:
                shooterMotorLeft.set(ControlMode.PercentOutput, 0.70);
                desiredPivotAngle = -.947 + 0;
                break;
            case HIGH:
                desiredPivotAngle = 1.0/5 - .947;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.90);
                break;
            case MID:
                desiredPivotAngle = -.947 + 1.0 / 8;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.70);
                break;
            case LOW:
                desiredPivotAngle = -.947;
                shooterMotorLeft.set(ControlMode.PercentOutput, -0.60);
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
