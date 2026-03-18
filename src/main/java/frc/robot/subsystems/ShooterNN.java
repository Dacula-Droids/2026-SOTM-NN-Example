// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class ShooterNN {
    private double[][] W0, W1, W2, W3, W4;
    private double[] b0, b1, b2, b3, b4;

    // Create buffers for slight optimization, not sure how much this actually helps
    private double[] nnReadyInputs;
    private double[] h0;
    private double[] h1;
    private double[] h2;
    private double[] h3;
    private double[] out;

    // Stats used to normalize inputs and denormalize outputs
    private double[] X_mean; 
    private double[] X_stdv; 
    private double[] y_mean; 
    private double[] y_stdv;

    private final double[] TARGET_POS = {4.6256, 4.0347, 1.8288}; // Center of hub
    private final double[] TARGET_POS_2D = {4.6256, 4.0347};
    private final double HUB_RADIUS = 0.529971;

    private final ShooterType shooterType;

    private final double EXIT_HEIGHT;
    private final double MIN_SHOOT_DISTANCE;
    private final double FIXED_HOOD_ANGLE;
    private final double FIXED_LAUNCH_SPEED;

    @SuppressWarnings("unchecked")
    public ShooterNN(String folder) throws Exception {
        String base = Filesystem.getDeployDirectory().getAbsolutePath();
        
        W0 = loadMatrix(base + "/"+folder+"/layer0_W.csv");
        b0 = loadVector(base + "/"+folder+"/layer0_b.csv");

        W1 = loadMatrix(base + "/"+folder+"/layer1_W.csv");
        b1 = loadVector(base + "/"+folder+"/layer1_b.csv");

        W2 = loadMatrix(base + "/"+folder+"/layer2_W.csv");
        b2 = loadVector(base + "/"+folder+"/layer2_b.csv");

        W3 = loadMatrix(base + "/"+folder+"/layer3_W.csv");
        b3 = loadVector(base + "/"+folder+"/layer3_b.csv");

        W4 = loadMatrix(base + "/"+folder+"/layer4_W.csv");
        b4 = loadVector(base + "/"+folder+"/layer4_b.csv");

        nnReadyInputs = new double[W0[0].length];

        h0 = new double[W0.length];
        h1 = new double[W1.length];
        h2 = new double[W2.length];
        h3 = new double[W3.length];
        out = new double[W4.length];

        var specifications = loadJson(base + "/"+folder+"/specifications.json");
        X_mean = ((List<Double>)specifications.get("X_mean")).stream()
                                         .mapToDouble(Double::doubleValue)
                                         .toArray();
        X_stdv = ((List<Double>)specifications.get("X_stdv")).stream()
                                         .mapToDouble(Double::doubleValue)
                                         .toArray();
        y_mean = ((List<Double>)specifications.get("y_mean")).stream()
                                         .mapToDouble(Double::doubleValue)
                                         .toArray();
        y_stdv = ((List<Double>)specifications.get("y_stdv")).stream()
                                         .mapToDouble(Double::doubleValue)
                                         .toArray();
        shooterType = ShooterType.valueOf((String)specifications.get("SHOOTER_TYPE"));                                         
        
        EXIT_HEIGHT = (double)specifications.get("EXIT_HEIGHT");
        switch (shooterType) {
            case VarPitch_VarSpeed:
                MIN_SHOOT_DISTANCE = 0;
                FIXED_HOOD_ANGLE = 0;
                FIXED_LAUNCH_SPEED = 0;
                break;
            case VarPitch_FixedSpeed:
                FIXED_LAUNCH_SPEED = (double)specifications.get("FIXED_LAUNCH_SPEED");
                FIXED_HOOD_ANGLE = 0;
                MIN_SHOOT_DISTANCE = 0;
                break;
            case FixedPitch_VarSpeed:
                MIN_SHOOT_DISTANCE = (double)specifications.get("MIN_SHOOT_DISTANCE");
                FIXED_HOOD_ANGLE = (double)specifications.get("FIXED_HOOD_ANGLE");
                FIXED_LAUNCH_SPEED = 0;
                break;
            default:
                MIN_SHOOT_DISTANCE = 0;
                FIXED_HOOD_ANGLE = 0;
                FIXED_LAUNCH_SPEED = 0;
                break;
        }
    }

    private void forward(double[] input) { 
        // Normalize inputs
        for (int i = 0; i < input.length; i++){
            input[i] = (input[i] - X_mean[i])/X_stdv[i];
        }

        matVecRelu(W0, input, b0, h0, true);
        matVecRelu(W1, h0, b1, h1, true);
        matVecRelu(W2, h1, b2, h2, true);
        matVecRelu(W3, h2, b3, h3, true);
        matVecRelu(W4, h3, b4, out, false);

        //Denormalize outputs
        for (int i = 0; i < out.length; i++){
            out[i] = (out[i] * y_stdv[i]) + y_mean[i];
        }
    }

    private void convertInputsToNNReadyForm(double[] inputs){
        double x = inputs[0];
        double y = inputs[1];
        double vx = inputs[2];
        double vy = inputs[3];

        double distance = Math.sqrt(Math.pow(TARGET_POS[0]-x,2) + Math.pow(TARGET_POS[1]-y,2));
        double theta = Math.atan((TARGET_POS[1]-y)/(TARGET_POS[0]-x));

        double vRadial = vx*Math.cos(theta) + vy*Math.sin(theta);
        double vTangential = -vx*Math.sin(theta) + vy*Math.cos(theta);

        nnReadyInputs[0] = distance;
        nnReadyInputs[1] = theta;
        nnReadyInputs[2] = vRadial;
        nnReadyInputs[3] = vTangential;
    }

    /**
     * Runs the given input/state through the NN and outputs the shot params specific to your {@code ShooterType}.
     *
     * <p>Make sure not to give inputs outside of the training data, e.g outside of the shooting area</p>
     *
     * @param robotPose The pose of your robot in the shooting area. Make sure you take into account alliance coordinate systems beforehand.
     * @param robotVelocity The velocity of your robot. Make sure you take into account alliance coordinate systems beforehand.
     * @param robotToExitTransform This is the transform from the center of your robot to where the ball will <b>exit</b> the shooter at, 
     * <b>NOT</b> the position of the shooter itself. Only the (x,y) are used, the height (z) is already baked into the model; {@code .getExitHeight()}
     * @return 
     * <ul>
     *  <li>{@code [heading, launch_angle, launch_speed]} if {@code ShooterType.VarPitch_VarSpeed}</li>
     *  <li>{@code [heading, launch_angle]} if {@code ShooterType.VarPitch_FixedSpeed}</li>
     *  <li>{@code [heading, launch_speed]} if {@code ShooterType.FixedPitch_VarSpeed}</li>
     *  <li>{@code heading}: <b>[-pi, pi]</b>, {@code launch_angle}: <b>[0, pi/2]</b>, {@code launch_speed}: <b>[0, 20] m/s</b>
     * </ul>
     */
    public double[] getShotParams(Pose2d robotPose, ChassisSpeeds robotVelocity, Transform2d robotToExitTransform){
        Pose2d fuelPose2d = robotPose.transformBy(robotToExitTransform);
        this.convertInputsToNNReadyForm(new double[]{fuelPose2d.getX(), fuelPose2d.getY(), robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond});
        this.forward(nnReadyInputs);
        double[] rawShotParams = out;
        double[] shotParams = new double[rawShotParams.length-1];
        shotParams[0] = Math.atan2(clamp(rawShotParams[1], -1., 1.), clamp(rawShotParams[0], -1., 1.));

        switch (this.shooterType) {
            case VarPitch_VarSpeed:
                shotParams[1] = clamp(rawShotParams[2], 0, Math.PI/2);
                shotParams[2] = clamp(rawShotParams[3], 0, 20);
                break;
            case VarPitch_FixedSpeed:
                shotParams[1] = clamp(rawShotParams[2], 0, Math.PI/2);
                break;
            case FixedPitch_VarSpeed:
                shotParams[1] = clamp(rawShotParams[2], 0, 20);
                break;
        }

        return shotParams;
    }

    /**
     * Checks if the pos is within the shooting area minus the wall and hub margins. This is <b>highly recommended</b> as it makes sure that you won't get bad outputs from the NN because of inputs outside or near the edge of the data it was trained on.
     *
     * <p>For shooters of type {@code ShooterType.FixedPitch_VarSpeed}, it makes sure that the pose is further than the required minimum distance away. Otherwise it makes sure that the pose is further than the {@code HUB_RADIUS+HUB_MARGIN}</p>
     *
     * @param pose2d The pose of your robot
     * @param robotToExitTransform This is the transform from the center of your robot to where the ball will <b>exit</b> the shooter at, <b>NOT</b> the position of the shooter itself. Only the (x,y) are used, the height (z) is already baked into the model; {@code .getExitHeight()}
     * @param wallMargin Padding from the wall, distances within a wall margin distance away from the wall will be considered invalid
     * @param hubMargin Padding from the hub, distances within a hub margin distance away from the hub will be considered invalid
     */
    public boolean isPosValid(Pose2d pose2d, Transform2d robotToExitTransform, Distance wallMargin, Distance hubMargin){
        pose2d = pose2d.transformBy(robotToExitTransform);
        boolean min_shoot_distance_required = this.shooterType == ShooterType.FixedPitch_VarSpeed;
        return (
            (pose2d.getX() > wallMargin.in(Meters)) && 
            (pose2d.getX() < TARGET_POS_2D[0] - wallMargin.in(Meters)) &&
            (pose2d.getY() > wallMargin.in(Meters)) && 
            (pose2d.getY() < TARGET_POS_2D[1] - wallMargin.in(Meters)) &&
            (
                new Translation2d(TARGET_POS_2D[0], TARGET_POS_2D[1]).getDistance(pose2d.getTranslation()) > 
                (min_shoot_distance_required ? this.getMinShootDistance():(HUB_RADIUS+hubMargin.in(Meters)))
            )
        );
    }

    public ShooterType getShooterType(){
        return this.shooterType;
    }

    /**
     * Gets the minimum distance needed to shoot and score with a {@code ShooterType.FixedPitch_VarSpeed}.
     *
     * <p>Shooters of type {@code ShooterType.FixedPitch_VarSpeed} have to be a minimum distance away to shoot. This is a conservative estimate from the kinematic equations. <b>DO NOT</b> attempt to shoot any closer than this as it is not in the NN's training dataset</p>
     *
     * @return Minumum distance in <b>meters</b> need to score for {@code ShooterType.FixedPitch_VarSpeed}; Otherwise <b>0</b>
     */
    public double getMinShootDistance(){
        return this.MIN_SHOOT_DISTANCE;
    }

    /**
     * Gets the fixed hood angle that the NN was trained on for shooters of type {@code ShooterType.FixedPitch_VarSpeed}
     *
     *
     * @return The fixed hood angle in <b>rad</b> that the NN was trained on for {@code ShooterType.FixedPitch_VarSpeed}; Otherwise <b>0</b>
     */
    public double getFixedHoodAngle(){
        return this.FIXED_HOOD_ANGLE;
    }

    /**
     * Gets the fixed launch speed that the NN was trained on for shooters of type {@code ShooterType.VarPitch_FixedSpeed}
     *
     *
     * @return The fixed launch speed in <b>m/s</b> that the NN was trained on for {@code ShooterType.VarPitch_FixedSpeed}; Otherwise <b>0</b>
     */
    public double getFixedLaunchSpeed(){
        return this.FIXED_LAUNCH_SPEED;
    }

    /**
     * Gets the fixed exit height that the NN was trained on
     *
     * @return the fixed exit height in <b>meters</b> that the NN was trained on
     */
    public double getExitHeight(){
        return this.EXIT_HEIGHT;
    }


    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static void matVecRelu(
        double[][] W,
        double[] x,
        double[] b,
        double[] y,
        boolean relu
    ) {
        int rows = W.length;
        int cols = x.length;

        for (int i = 0; i < rows; i++) {
            double sum = b[i];
            double[] Wi = W[i];

            for (int j = 0; j < cols; j++) {
                sum += Wi[j] * x[j];
            }

            if (relu && sum < 0) sum = 0;
            y[i] = sum;
        }
    }

    private static double[][] loadMatrix(String filename) throws IOException {    
        List<double[]> rows = new ArrayList<>();

        List<String> lines = Files.readAllLines(Paths.get(filename));
        
        for (String line : lines) {
            String[] parts = line.split(",");
            double[] row = new double[parts.length];

            for (int i = 0; i < parts.length; i++) {
                row[i] = Double.parseDouble(parts[i]);
            }

            rows.add(row);
        }

        return rows.toArray(new double[0][]);
    }

    private static double[] loadVector(String filename) throws IOException {
       List<String> lines = Files.readAllLines(Paths.get(filename));

        double[] vec = new double[lines.size()];

        for (int i = 0; i < lines.size(); i++) {
            vec[i] = Double.parseDouble(lines.get(i));
        }

        return vec;
    }

    private static Map<String, Object> loadJson(String fileName) {
        Path deployPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
        ObjectMapper mapper = new ObjectMapper();
        try {
            @SuppressWarnings("unchecked")
            Map<String, Object> data = mapper.readValue(new File(deployPath.toString()), Map.class);
            return data;
        } catch (IOException e) {
            DriverStation.reportError("Error loading JSON file: " + fileName + " - " + e.getMessage(), true);
            return null;
        }
    }
}
