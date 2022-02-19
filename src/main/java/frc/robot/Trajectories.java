package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

import java.util.List;

public class Trajectories {

  TrajectoryConfig config;
  TrajectoryConfig reversedConfig;

  public Trajectory example;
  public Trajectory tarmacToBall;

  public Trajectories(TrajectoryConfig config, TrajectoryConfig reversedConfig) {
    this.config = config;
    this.reversedConfig = reversedConfig;

    example =
        TrajectoryGenerator.generateTrajectory(
            // Start at (1, 2) facing the +X direction
            new Pose2d(1, 2, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4, 2, new Rotation2d(0)),
            // Pass config
            config);

    tarmacToBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1.484, 0.156, new Rotation2d(Units.degreesToRadians(-3.4)))),
            config);
  }
}
