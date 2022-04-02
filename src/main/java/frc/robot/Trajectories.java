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
            List.of(new Translation2d(8.6, 7.6), new Translation2d(10.5, 7.73)),
            new Pose2d(12.50, 6.85, new Rotation2d(Units.degreesToRadians(-60))),
            AutoConstants.config);

    public static final Pose2d shootPose =
        new Pose2d(
            getLastPose(initToWallToShoot).getTranslation(),
            new Rotation2d(Units.degreesToRadians(-152.3)));

    public static final Pose2d HPSpose = 
    new Pose2d(15.27, 7.02, new Rotation2d(Units.degreesToRadians(43)));

    public static final Trajectory slightForward =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                shootPose,
                getColinearPose(shootPose, 0.35)),
            AutoConstants.config);

    public static final Trajectory ShootToHPS =
        TrajectoryGenerator.generateTrajectory(
            getLastPose(slightForward),
            List.of(new Translation2d(12.34, 5.73)),
            HPSpose,
            AutoConstants.config);

    public static final Trajectory HPStoShoot =
        TrajectoryGenerator.generateTrajectory(
            HPSpose,
            List.of(new Translation2d(12.75, 5.6)),
            shootPose,
            AutoConstants.reversedConfig);

    public static final Trajectory HPStoFarShoot =
        // TrajectoryGenerator.generateTrajectory(
        //     HPSpose,
        //     List.of(new Translation2d(14.5, 6.0), new Translation2d(13.7, 6.2)),
        //     new Pose2d(13.82, 7.0, new Rotation2d(Units.degreesToRadians(-155))),
        //     AutoConstants.reversedConfig);

            TrajectoryGenerator.generateTrajectory(
            List.of(
                HPSpose,    
                getColinearPose(HPSpose, -1.4)),
                
                
            AutoConstants.reversedConfig);
            //new Pose2d(14.40, 5.95, new Rotation2d(Units.degreesToRadians(43.7)))),
            // new Pose2d(14.39, 5.94, new Rotation2d(Units.degreesToRadians(175.4)))),

  }
}
