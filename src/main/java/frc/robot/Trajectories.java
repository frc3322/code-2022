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

  public static final Trajectory oneMeterForward =
      TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0))),
          AutoConstants.config);

  public static final class FourBallAuto {
    public static final Pose2d initPose =
      new Pose2d(new Translation2d(9.764, 5.583), new Rotation2d(Units.degreesToRadians(24)));

    public static final Trajectory tarmacToBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(initPose.getTranslation(), new Rotation2d(Units.degreesToRadians(-5.5))),
                new Pose2d(11.18, 5.959, new Rotation2d(Units.degreesToRadians(60.562)))),
            AutoConstants.config);

    public static final Trajectory ballToShoot = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                getLastPose(tarmacToBall),
                new Pose2d(9.754, 6.844, new Rotation2d(Units.degreesToRadians(-120)))),
            AutoConstants.config);

    public static final Trajectory tarmacToShoot = 
        TrajectoryGenerator.generateTrajectory(
            List.of(tarmacToBall.getInitialPose(), ballToShoot.getInitialPose(), getLastPose(ballToShoot)), AutoConstants.config);

    public static final Trajectory shootToHumanPlayer =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(
                    getLastPose(ballToShoot).getTranslation(),
                    new Rotation2d(Units.degreesToRadians(-12))),
                new Pose2d(15.051, 6.827, new Rotation2d(Units.degreesToRadians(43)))),
            AutoConstants.config);

    public static final Trajectory humanPlayerToShoot =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                getLastPose(shootToHumanPlayer),
                new Pose2d(getLastPose(ballToShoot).getTranslation(), new Rotation2d(Units.degreesToRadians(-12)))),
            AutoConstants.reversedConfig);
  }

  private static Pose2d getLastPose(Trajectory trajectory){
      return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
  }
}
