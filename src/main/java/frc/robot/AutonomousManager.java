package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private static final AutonomousOption defaultAuto = AutonomousOption.CIRCLE;

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration;
    LoggedReceiver startPosition;
    LoggedReceiver gamePieces;
    LoggedReceiver shouldClimb;

    private String previousStartPosition = defaultAuto.startPosition.name();
    private int previousGamePieces = defaultAuto.gamePieces;

    private SwerveAutoBuilder autoBuilder;

    private List<PathPlannerTrajectory> chosenAuto = defaultAuto.getPath();

    SwerveDriveSubsystem swerveDriveSubsystem;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();

        initializeNetworkTables();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("levelChargeStation", swerveDriveSubsystem.levelChargeStationCommandDestiny());

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(1.2, 0.0, 0.001),
                (ChassisSpeeds velocity) -> swerveDriveSubsystem.setVelocity(velocity, false, false),
                eventMap,
                true,
                swerveDriveSubsystem);
    }

    public void update() {
        var newStartPosition = startPosition.getString();
        var newGamePieces = gamePieces.getInteger();

        // Only update the chosen auto if a different option has been chosen
        if (previousStartPosition != newStartPosition || previousGamePieces != newGamePieces) {
            // Match the auto based on the dashboard configuration
            List<AutonomousOption> options = Stream.of(AutonomousOption.values())
                    .filter(option ->
                            option.startPosition.name().equals(newStartPosition) && option.gamePieces == newGamePieces)
                    .toList();

            if (options.size() == 1) chosenAuto = options.get(0).getPath();
            else chosenAuto = defaultAuto.getPath();

            // Determine all of the game piece options for this starting position
            long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                    .filter(option -> option.startPosition.name().equals(newStartPosition))
                    .mapToLong(option -> option.gamePieces)
                    .toArray();

            Logger.log("/Autonomous/Game Piece Options", gamePieceOptions).alwaysNT();

            previousStartPosition = newStartPosition;
            previousGamePieces = (int) newGamePieces;
        }
    }

    public Command getAutonomousCommand() {
        Command chosenPathCommand = autoBuilder.fullAuto(chosenAuto);

        var chosenWaitDuration = waitDuration.getInteger();

        if (chosenWaitDuration > 0) chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));

        return chosenPathCommand;
    }

    private void initializeNetworkTables() {
        waitDuration = Logger.tunable("/Autonomous/Wait Duration", 0.0);
        startPosition = Logger.tunable(
                "/Autonomous/Start Position", defaultAuto.startPosition.name()); // 0 = Left, 1 = Center, 2 = Right
        gamePieces = Logger.tunable("/Autonomous/Game Pieces", defaultAuto.gamePieces);
        shouldClimb = Logger.tunable("/Autonomous/Should Climb", true);

        Logger.log("/Autonomous/Start Position Options", getStartingLocations()).alwaysNT();

        // Determine all of the game piece options for this starting position
        long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                .filter(option -> option.startPosition.equals(defaultAuto.startPosition))
                .mapToLong(option -> option.gamePieces)
                .toArray();

        Logger.log("/Autonomous/Game Piece Options", gamePieceOptions).alwaysNT();
    }

    private enum AutonomousOption {
        // OPEN_PLACE1ANDCLIMB(StartingLocation.OPEN, 1, "open_place1andclimb", new PathConstraints(5, 5)),
        // OPEN_PLACE2(StartingLocation.OPEN, 2, "open_place2", new PathConstraints(4, 3)),
        // OPEN_PLACE3(StartingLocation.OPEN, 3, "open_place3", new PathConstraints(4, 4)),
        STATION_PLACE1ANDCLIMB(StartingLocation.STATION, 1, "station_place1andclimb", new PathConstraints(3, 2.25)),
        OPEN_PLACE1ANDTAXI(StartingLocation.OPEN, 1, "open_place1andtaxi", new PathConstraints(3, 3.25)),
        CABLE_PLACE1ANDTAXI(StartingLocation.CABLE, 1, "cable_place1andtaxi", new PathConstraints(2.85, 2.5)),
        CIRCLE(StartingLocation.STATION, 2, "circle", new PathConstraints(2.75, 2.25));
        // CABLE_PLACE1ANDCLIMB(StartingLocation.CABLE, 1, "cable_place1andclimb", new PathConstraints(5, 5)),
        // CABLE_PLACE2(StartingLocation.CABLE, 2, "cable_place2", new PathConstraints(4, 3)),
        // CABLE_PLACE3(StartingLocation.CABLE, 3, "cable_place3", new PathConstraints(3.5, 3));

        private List<PathPlannerTrajectory> path;
        private String pathName;
        private PathConstraints constraints;
        public StartingLocation startPosition;
        public int gamePieces;

        private AutonomousOption(
                StartingLocation startPosition, int gamePieces, String pathName, PathConstraints constraints) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.pathName = pathName;
            this.constraints = constraints;
        }

        public List<PathPlannerTrajectory> getPath() {
            // Lazy load the path
            if (path == null) path = PathPlanner.loadPathGroup(pathName, constraints);

            return path;
        }
    }

    private enum StartingLocation {
        OPEN,
        STATION,
        CABLE
    }

    public static String[] getStartingLocations() {
        return Stream.of(StartingLocation.values()).map(StartingLocation::name).toArray(String[]::new);
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}
