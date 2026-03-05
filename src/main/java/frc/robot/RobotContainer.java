// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    // If you want to time how long it takes the NN to run on avg. If for some reason it's slow (shouldn't be), try warming up by calling the .getShotParams a few hundred times to get the JIT to compile.
    // long startTime = System.nanoTime();
    // double[] shotParams = shooterNN.getShotParams(robotPose2d, robotVelocity, robotToExitTransform);
    // // for (int i = 0; i < 1000; i++){
    // //   shotParams = shooterNN.getShotParams(robotPose2d, robotVelocity, robotToExitTransform);
    // // }
    // long endTime = System.nanoTime();
    // long duration = (endTime - startTime); // In nanoseconds
    // System.out.println("Function took: " + (duration / 1e6)/1000 + " ms");

    Pose2d robotPose2d = new Pose2d(2,2, new Rotation2d());
    ChassisSpeeds robotVelocity = new ChassisSpeeds(1,2,0);
    
    Optional<double[]> shotParams = shooterSubsystem.getShotParams(robotPose2d, robotVelocity);
    // Use this for ShooterType.FixedPitch_VarSpeed
    System.out.println(
      "Heading: "+ Units.radiansToDegrees(shotParams.get()[0]) + 
      " Launch Speed: "+shotParams.get()[1]
    );
    
    // Use this for ShooterType.VarPitch_VarSpeed
    // System.out.println(
    //   "Heading: "+ Units.radiansToDegrees(shotParams.get()[0]) + 
    //   " Launch Angle: "+ Units.radiansToDegrees(shotParams.get()[1]) +
    //   " Launch Speed: "+shotParams.get()[2]
    // );

    // Use this for ShooterType.VarPitch_FixedSpeed
    // System.out.println(
    //   "Heading: "+ Units.radiansToDegrees(shotParams.get()[0]) + 
    //   " Launch Angle: "+ Units.radiansToDegrees(shotParams.get()[1])
    // );

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
