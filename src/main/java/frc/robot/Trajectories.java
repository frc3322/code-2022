package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import java.util.List;

public final class Trajectories {

  public static final Pose2d initPose =
      new Pose2d(new Translation2d(9.764, 5.583), new Rotation2d(Units.degreesToRadians(24)));

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

  public static final class fourBallAuto {
    public static final Trajectory tarmacToBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(1.484, 0.156, new Rotation2d(Units.degreesToRadians(-3.4)))),
            AutoConstants.config);

    public static final Trajectory ballToHumanPlayer =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(1.484, 0.156, new Rotation2d(Units.degreesToRadians(-3.4))),
                new Pose2d(5.533, -0.774, new Rotation2d(Units.degreesToRadians(22.52)))),
            AutoConstants.config);

    public static final Trajectory humanPlayerToShoot =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(5.533, -0.774, new Rotation2d(Units.degreesToRadians(22.52))),
                new Pose2d(1.701, -0.616, new Rotation2d(Units.degreesToRadians(0)))),
            AutoConstants.reversedConfig);
  }

  public static final class fourBallAutoAdjusted {
    public static final Trajectory tarmacToBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                initPose,
                new Pose2d(11.148, 6.227, new Rotation2d(Units.degreesToRadians(26.294)))),
            AutoConstants.config);

    public static final Trajectory ballToHumanPlayer =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(
                    tarmacToBall
                        .getStates()
                        .get(tarmacToBall.getStates().size() - 1)
                        .poseMeters
                        .getTranslation(),
                    new Rotation2d(0)),
                new Pose2d(15.051, 6.827, new Rotation2d(Units.degreesToRadians(45)))),
            AutoConstants.config);

    public static final Trajectory humanPlayerToShoot =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                ballToHumanPlayer
                    .getStates()
                    .get(ballToHumanPlayer.getStates().size() - 1)
                    .poseMeters,
                new Pose2d(10.712, 6.390, new Rotation2d(Units.degreesToRadians(37)))),
            AutoConstants.reversedConfig);
  }

  public static final class fourBallAutoTransformed {

    private static final Transform2d transform =
        new Transform2d(initPose.getTranslation(), initPose.getRotation());

    public static final Trajectory tarmacToBall = fourBallAuto.tarmacToBall.transformBy(transform);

    private static final Pose2d initPoseTwo =
        tarmacToBall.getStates().get(tarmacToBall.getStates().size() - 1).poseMeters;
    private static final Transform2d transformTwo =
        new Transform2d(fourBallAuto.ballToHumanPlayer.getInitialPose(), initPoseTwo);

    public static final Trajectory ballToHumanPlayer =
        fourBallAuto.ballToHumanPlayer.transformBy(transformTwo);

    private static final Pose2d initPoseThree =
        ballToHumanPlayer.getStates().get(ballToHumanPlayer.getStates().size() - 1).poseMeters;
    private static final Transform2d transformThree =
        new Transform2d(fourBallAuto.humanPlayerToShoot.getInitialPose(), initPoseThree);

    public static final Trajectory humanPlayerToShoot =
        fourBallAuto.humanPlayerToShoot.transformBy(transformThree);
  }
}
