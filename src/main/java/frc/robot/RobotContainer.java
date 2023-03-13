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

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(35, -16, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(35, -16, 0);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis, true));

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");
        leftDriveController.nameLeftTopRight("Reset Pose");

        leftDriveController.nameTrigger("Run Gripper");
        leftDriveController.nameTrigger("Eject Gripper");

        // Leveling
        leftDriveController.getLeftBottomLeft().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommandDestiny());

        leftDriveController.getLeftBottomMiddle().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        leftDriveController.nameLeftBottomLeft("Level Charge Station");
        leftDriveController.nameLeftBottomMiddle("Lock Wheels");

        // Auto Aim Behaviors
        // leftDriveController
        //         .getRightThumb()
        //         .whileTrue(new AssistToGridCommand(
        //                 swerveDriveSubsystem,
        //                 visionSubsystem,
        //                 lightsSubsystem,
        //                 getTargetPoseSupplier(),
        //                 this::getDriveForwardAxis));
        // leftDriveController
        //         .getLeftThumb()
        //         .whileTrue(new AimAtPoseCommand(
        //                         swerveDriveSubsystem,
        //                         getTargetAimPoseSupplier(),
        //                         this::getDriveForwardAxis,
        //                         this::getDriveStrafeAxis)
        //                 .alongWith(visionSubsystem.customLimelightModeCommand()));

        // leftDriveController
        //         .getLeftThumb()
        //         .whileTrue(new AssistedMLPickupCommand(
        //                 swerveDriveSubsystem,
        //                 visionSubsystem,
        //                 this::getDriveForwardAxis,
        //                 this::getDriveStrafeAxis,
        //                 this::getDriveRotationAxis)); // before running set the pipeline
        // leftDriveController.nameLeftThumb("ML Pickup");

        leftDriveController.nameRightThumb("Assist to Pose");
        leftDriveController.nameRightThumb("Assisted ML Aim");

        /* Set right joystick bindings */
        rightDriveController.getRightBottomMiddle().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
        rightDriveController.getRightBottomRight().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
        rightDriveController.nameRightBottomMiddle("Characterize Forwards");
        rightDriveController.nameRightBottomMiddle("Characterize Backwards");

        // rightDriveController
        //         .getBottomThumb()
        //         .whileTrue(new AssistedMLPickupCommand(
        //                 swerveDriveSubsystem,
        //                 visionSubsystem,
        //                 this::getDriveForwardAxis,
        //                 this::getDriveStrafeAxis,
        //                 this::getDriveRotationAxis)); // before running set the pipeline
        // rightDriveController.nameBottomThumb("ML Pickup");

        // Cardinal drive commands (inverted since arm is back of robot)
        // rightDriveController
        //         .getPOVUp()
        //         .whileTrue(swerveDriveSubsystem.cardinalCommand(
        //                 Rotation2d.fromDegrees(180), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        // rightDriveController
        //         .getPOVRight()
        //         .whileTrue(swerveDriveSubsystem.cardinalCommand(
        //                 Rotation2d.fromDegrees(90), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        // rightDriveController
        //         .getPOVDown()
        //         .whileTrue(swerveDriveSubsystem.cardinalCommand(
        //                 Rotation2d.fromDegrees(0), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        // rightDriveController
        //         .getPOVLeft()
        //         .whileTrue(swerveDriveSubsystem.cardinalCommand(
        //                 Rotation2d.fromDegrees(-90), this::getDriveForwardAxis, this::getDriveStrafeAxis));

        // operatorController.getRightBumper().whileTrue(intakeSubsystem.handoffCommand());

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public double getDriveForwardAxis() {
        return forwardRateLimiter.calculate(
                -square(deadband(leftDriveController.getYAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveStrafeAxis() {
        return strafeRateLimiter.calculate(
                -square(deadband(leftDriveController.getXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveRotationAxis() {
        return -square(deadband(rightDriveController.getXAxis().getRaw(), 0.05))
                * Constants.SwerveConstants.maxAngularVelocity
                * 0.75;
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }
}