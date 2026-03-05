// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  ShooterNN shooterNN;

  // Should probably put these in a contansts file
  Transform2d robotToExitTransform = new Transform2d(0,0, new Rotation2d());
  Distance FIELD_X = Units.Meters.of(16.513048); // Length of full field in the x
  Distance FIELD_Y = Units.Meters.of(8.042656); // Length of full field in the y
  Pose2d redAllianceOrigin = new Pose2d(FIELD_X.magnitude(), FIELD_Y.magnitude(), Rotation2d.fromDegrees(180));
  Distance WALL_MARGIN = Units.Meters.of(1);
  Distance HUB_MARGIN = Units.Meters.of(1);

  public ShooterSubsystem() {
    try {
      //Pass in the name of the folder (must be in deploy directory) containing you layer Weights and Biases along with the specifications.json
      shooterNN = new ShooterNN("team2181fixed47degv1"); 
    } catch (Exception e){}
  }

  // Assumes that robotPose2d is given relative to blue alliance origin. If this is not the case, handle it.
  public Optional<double[]> getShotParams(Pose2d robotPose2d, ChassisSpeeds robotVelocity){
    var alliance = DriverStation.getAlliance();

    //Convert robotPose2d and robotvelocity relative to red alllince if on red
    if (DriverStation.getAlliance().isPresent() && alliance.get() == Alliance.Red) {
      robotPose2d = robotPose2d.relativeTo(redAllianceOrigin);
      robotVelocity = new ChassisSpeeds(-robotVelocity.vxMetersPerSecond, -robotVelocity.vyMetersPerSecond, robotVelocity.omegaRadiansPerSecond);
    }

    // Checks if the pos is within the shooting area minus the wall and hub margins. 
    // This is highly recommended as it makes sure that you won't get bad outputs from the NN because of inputs outside or near the edge of the data it was trained on.
    if (!shooterNN.isPosValid(robotPose2d, robotToExitTransform, WALL_MARGIN, HUB_MARGIN)) return Optional.empty();

    //for ShooterType.VarPitch_VarSpeed, returns: [heading, launch_angle, launch_speed]
    //for ShooterType.VarPitch_FixedSpeed, returns: [heading, launch_angle]
    //for ShooterType.FixedPitch_VarSpeed, returns: [heading, launch_speed]
    double[] shotParams = shooterNN.getShotParams(robotPose2d, robotVelocity, robotToExitTransform);
    double req_heading = shotParams[0];


    // If you don't have a Yaw controlled turret and are using your swerve drive to set your heading, 
    // you should reject large heading changes as the latency will likely be too large.
    // If you do this, I recommend having your robot already aiming somwhere near the hub before calling this method as it minimizes latency
    if (Math.abs(req_heading - robotPose2d.getRotation().getRadians()) > Math.PI/2) return Optional.empty();
    
    return Optional.of(shotParams);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
