package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(35, -35, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(35, -35, 0);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {

        // /* Set default commands */
        // swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
        //         this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis, true));
        
        // shooterSubsystem.setDefaultCommand(shooterSubsystem.setDisabledCommand());

        // /* Set left joystick bindings */
        // leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        // leftDriveController
        //         .getLeftTopRight()
        //         .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
        // leftDriveController.nameLeftTopLeft("Reset Gyro Angle");

        // // Leveling
        // leftDriveController.getLeftBottomLeft().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommandDestiny());

        // leftDriveController.getLeftBottomMiddle().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        // leftDriveController.nameLeftBottomLeft("Level Charge Station");
        // leftDriveController.nameLeftBottomMiddle("Lock Wheels");

        /* Set right joystick bindings */
        rightDriveController.getRightBottomMiddle().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
        rightDriveController.getRightBottomRight().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
        rightDriveController.nameRightBottomMiddle("Characterize Forwards");
        rightDriveController.nameRightBottomMiddle("Characterize Backwards");
        rightDriveController.getLeftThumb().whileTrue(shooterSubsystem.shootHighCommand());
        rightDriveController.getRightThumb().whileTrue(shooterSubsystem.shootMidCommand());
        rightDriveController.getBottomThumb().whileTrue(shooterSubsystem.shootLowCommand());


        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();

        // Cardinal drive commands (inverted since the arm is on the back of the robot)
        rightDriveController
                .getPOVUp()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(180), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVRight()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(90), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVDown()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(0), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVLeft()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(-90), this::getDriveForwardAxis, this::getDriveStrafeAxis));

    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public double getDriveForwardAxis() {
        return //forwardRateLimiter.calculate(
                -square(deadband(leftDriveController.getYAxis().getRaw(), 0.05))
                        * Constants.SwerveConstants.maxSpeed;//);
    }

    public double getDriveStrafeAxis() {
        return //strafeRateLimiter.calculate(
                -square(deadband(leftDriveController.getXAxis().getRaw(), 0.05))
                        * Constants.SwerveConstants.maxSpeed;//);
    }

    public double getDriveRotationAxis() {
        return -cube(deadband(rightDriveController.getXAxis().getRaw(), 0.05))
                * Constants.SwerveConstants.maxAngularVelocity
                * 0.5;
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private static double cube(double value) {
        return Math.copySign(value * value * value, value);
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }
}
