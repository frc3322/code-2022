package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import java.util.List;

public final class Trajectories {

  public static final Trajectory example =
      TrajectoryGenerator.generateTrajectory(
          // Start at (1, 2) facing the +X direction
          new Pose2d(1, 2, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(4, 2, new Rotation2d(0)),
          // Pass config
          AutoConstants.config);

  public static final Trajectory straightForward =
      TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1.6, 0, new Rotation2d(0))),
          AutoConstants.config);

  public static final class RightSide {

    public static final Pose2d initPose =
        new Pose2d(new Translation2d(9.764, 5.583), new Rotation2d(Units.degreesToRadians(24)));

    public static final Trajectory tarmacToBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                initPose, new Pose2d(10.823, 6.079, new Rotation2d(Units.degreesToRadians(25.6)))),
            AutoConstants.config);

    public static final Trajectory ballToShoot =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                getLastPose(tarmacToBall),
                new Pose2d(9.693, 6.597, new Rotation2d(Units.degreesToRadians(-120)))),
            AutoConstants.config);

    public static final Trajectory tarmacToShoot =
        TrajectoryGenerator.generateTrajectory(
            tarmacToBall.getInitialPose(),
            List.of(ballToShoot.getInitialPose().getTranslation(), new Translation2d(11.0, 6.9)),
            getLastPose(ballToShoot),
            AutoConstants.config);

    public static final class FourBallAuto {

      public static final Trajectory shootToHumanPlayer =
          TrajectoryGenerator.generateTrajectory(
              List.of(
                  getLastPose(ballToShoot),
                  new Pose2d(15.02, 6.75, new Rotation2d(Units.degreesToRadians(43.7)))),
              AutoConstants.config);

      public static final Trajectory humanPlayerToShoot =
          TrajectoryGenerator.generateTrajectory(
              List.of(getLastPose(shootToHumanPlayer), getLastPose(tarmacToShoot)),
              AutoConstants.reversedConfig);
    }

    public static final class ThreeBallAuto {
      public static final Trajectory shootToWallBall =
          TrajectoryGenerator.generateTrajectory(
              getLastPose(ballToShoot),
              List.of(new Translation2d(8.986, 6.625)),
              new Pose2d(8.935, 7.5, new Rotation2d(Units.degreesToRadians(90))),
              AutoConstants.config);

      public static final Trajectory wallBallToShoot =
          TrajectoryGenerator.generateTrajectory(
              getLastPose(shootToWallBall),
              List.of(new Translation2d(8.986, 6.625)),
              getLastPose(ballToShoot),
              AutoConstants.reversedConfig);
    }
  }

  private static Pose2d getLastPose(Trajectory trajectory) {
    return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
  }

  public static final class FiveBall {

    public static final Pose2d initPose =
        new Pose2d(new Translation2d(8.316, 5.792), new Rotation2d(Units.degreesToRadians(113.8)));

    public static final Trajectory initToWallToShoot =
        TrajectoryGenerator.generateTrajectory(
            initPose,
            List.of(new Translation2d(8.6, 7.6)),
            new Pose2d(12.03, 6.6, new Rotation2d(Units.degreesToRadians(-60))),
            AutoConstants.config);

    public static final Pose2d shootPose =
        new Pose2d(getLastPose(initToWallToShoot).getTranslation(), new Rotation2d(Units.degreesToRadians(-152.3)));

    public static final Trajectory shootToBounce =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                getLastPose(initToWallToShoot),
                new Pose2d(13.7, 5.85, new Rotation2d(Units.degreesToRadians(80)))),
            AutoConstants.reversedConfig);

    public static final Trajectory bounceToHPS =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                getLastPose(shootToBounce),
                new Pose2d(15.15, 6.9, new Rotation2d(Units.degreesToRadians(43)))),
            AutoConstants.config);

    public static final Trajectory ShootToHPS =
        TrajectoryGenerator.generateTrajectory(
            shootPose,
            List.of(new Translation2d(12.34, 5.73)),
            getLastPose(bounceToHPS),
            AutoConstants.config);

    public static final Trajectory HPStoShoot =
        TrajectoryGenerator.generateTrajectory(
            getLastPose(bounceToHPS),
            List.of(new Translation2d(12.34, 5.73)),
            shootPose,
            AutoConstants.reversedConfig);
  }

  public static final class AltFourBall {
    public static final Pose2d initPose =
        new Pose2d(new Translation2d(8.316, 5.792), new Rotation2d(Units.degreesToRadians(113.8)));

    public static final Trajectory initToWallToShoot =
        TrajectoryGenerator.generateTrajectory(
            initPose,
            List.of(new Translation2d(8.45, 7.7)),
            new Pose2d(10.448, 6.349, new Rotation2d(Units.degreesToRadians(-122.9))),
            AutoConstants.config);

    public static final Trajectory shootToOutsideTarmacToHPS =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(
                    getLastPose(initToWallToShoot).getTranslation(),
                    new Rotation2d(Units.degreesToRadians(10.6))),
                new Pose2d(15.0, 6.95, new Rotation2d(Units.degreesToRadians(38)))),
            AutoConstants.config);

    public static final Trajectory HPSToFinalShoot =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                getLastPose(shootToOutsideTarmacToHPS),
                new Pose2d(12.033, 6.457, new Rotation2d(Units.degreesToRadians(-15)))),
            AutoConstants.reversedConfig);
  }
}
