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
              List.of(
                  getLastPose(shootToHumanPlayer),
                  getLastPose(tarmacToShoot)),
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

    //   public static final Trajectory tarmacToWallBall = null val;
    //   //wallBallToShoot
    //   public static final Trajectory wallBallShootToHPS = null;
    //   public static final Trajectory HPSToFinalShoot = null;

    
    }
  }

  private static Pose2d getLastPose(Trajectory trajectory) {
    return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
  }

  //
  public static final class AltFourBall {

    public static final Pose2d initPose =
    new Pose2d(new Translation2d(7.903, 6.411), new Rotation2d(Units.degreesToRadians(100)));

    public static final Trajectory initToWallToShoot = TrajectoryGenerator.generateTrajectory(
      List.of(initPose, new Pose2d(8.572, 7.702, new Rotation2d(Units.degreesToRadians(12))), new Pose2d(9.529,6.132, new Rotation2d(Units.degreesToRadians(-123)))),
      AutoConstants.config);

    public static final Trajectory shootToOutsideTarmacToHPS = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(9.592,6.298, new Rotation2d(Units.degreesToRadians(10.6))), new Pose2d(10.793,6.405, new Rotation2d(Units.degreesToRadians(-360))),new Pose2d(14.941,6.917, new Rotation2d(Units.degreesToRadians(38)))),
        AutoConstants.config);

    // public static final Trajectory HPSToFinalShoot = TrajectoryGenerator.generateTrajectory(
    //     List.of(getLastPose(shootToOutsideTarmacToHPS), new Pose2d(9.729,5.893, new Rotation2d(Units.degreesToRadians(37)))),
    //     AutoConstants.reversedConfig);

    public static final Trajectory HPSToFinalShoot = TrajectoryGenerator.generateTrajectory(
        List.of(getLastPose(shootToOutsideTarmacToHPS), new Pose2d(12.033,6.457, new Rotation2d(Units.degreesToRadians(-15)))),
        AutoConstants.reversedConfig);

    }
}
