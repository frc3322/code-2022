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

  public static final Trajectory straightForward =
      TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1.6, 0, new Rotation2d(0))),
          AutoConstants.config);

  private static Pose2d getLastPose(Trajectory trajectory) {
    return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
  }

  private static Pose2d getColinearPose(Pose2d initPose, double distance) {
    double theta = initPose.getRotation().getRadians();
    double xFinal = initPose.getTranslation().getX() + distance * Math.cos(theta);
    double yFinal = initPose.getTranslation().getY() + distance * Math.sin(theta);

    return new Pose2d(xFinal, yFinal, new Rotation2d(theta));
  }

  public static final class FiveBall {

    public static final Pose2d initPose =
        new Pose2d(new Translation2d(8.316, 5.792), new Rotation2d(Units.degreesToRadians(113.8)));

    public static final Trajectory initToWallToShoot =
        TrajectoryGenerator.generateTrajectory(
            initPose,
            List.of(new Translation2d(8.6, 7.6)),
            new Pose2d(12.25, 6.75, new Rotation2d(Units.degreesToRadians(-60))),
            AutoConstants.config);

    public static final Pose2d shootPose =
        new Pose2d(
            getLastPose(initToWallToShoot).getTranslation(),
            new Rotation2d(Units.degreesToRadians(-152.3)));

    public static final Pose2d HPSpose =
        new Pose2d(15.2, 6.95, new Rotation2d(Units.degreesToRadians(43)));

    public static final Trajectory slightForward =
        TrajectoryGenerator.generateTrajectory(
            List.of(shootPose, getColinearPose(shootPose, 0.5)), AutoConstants.config);

    public static final Trajectory ShootToHPS =
        TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(getLastPose(slightForward).getTranslation(), new Rotation2d(Units.degreesToRadians(-10))),
            HPSpose),
            AutoConstants.config);

    public static final Trajectory HPStoShoot =
        TrajectoryGenerator.generateTrajectory(
            List.of(HPSpose,
            new Pose2d(shootPose.getTranslation(), new Rotation2d(Units.degreesToRadians(-10)))),
            AutoConstants.reversedConfig);
  }
}
