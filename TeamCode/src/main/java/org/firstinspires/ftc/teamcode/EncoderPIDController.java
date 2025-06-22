package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * EncoderPIDController - A class to handle PID control for a robot with three dead wheel encoders
 * Two encoders are placed symmetrically on the left and right sides (y-axis tracking)
 * One encoder is placed on the back (x-axis tracking)
 */
@Config
public class EncoderPIDController {
    // PID coefficients for each axis
    public static class PIDCoefficients {
        public double kP = 0.0; // Proportional coefficient
        public double kI = 0.0; // Integral coefficient
        public double kD = 0.0; // Derivative coefficient
        public double integralSumLimit = 0.0; // Limit for integral sum to prevent windup
        
        public PIDCoefficients(double kP, double kI, double kD, double integralSumLimit) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.integralSumLimit = integralSumLimit;
        }
    }
    
    // Default PID coefficients for each axis
    public static PIDCoefficients X_PID_COEFFICIENTS = new PIDCoefficients(0.05, 0.0, 0.001, 0.5);
    public static PIDCoefficients Y_PID_COEFFICIENTS = new PIDCoefficients(0.05, 0.0, 0.001, 0.5);
    public static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0.05, 0.0, 0.001, 0.5);
    
    // Encoder objects
    private final Encoder leftEncoder, rightEncoder, backEncoder;
    
    // Encoder parameters
    private double inchesPerCount;
    private double encoderTrackWidth; // Distance between left and right encoders
    
    // PID state variables
    private double xError = 0.0;
    private double yError = 0.0;
    private double headingError = 0.0;
    
    private double xIntegralSum = 0.0;
    private double yIntegralSum = 0.0;
    private double headingIntegralSum = 0.0;
    
    private double lastXError = 0.0;
    private double lastYError = 0.0;
    private double lastHeadingError = 0.0;
    
    private ElapsedTime pidTimer = new ElapsedTime();
    
    // Target position
    private Pose2d targetPose = new Pose2d(0, 0, 0);
    
    // Current position
    private Pose2d currentPose = new Pose2d(0, 0, 0);
    
    /**
     * Constructor for EncoderPIDController
     * @param hardwareMap Hardware map from OpMode
     * @param inchesPerCount Conversion factor from encoder counts to inches
     * @param encoderTrackWidth Distance between left and right encoders in inches
     */
    public EncoderPIDController(HardwareMap hardwareMap, double inchesPerCount, double encoderTrackWidth) {
        this.inchesPerCount = inchesPerCount;
        this.encoderTrackWidth = encoderTrackWidth;
        
        // Initialize encoders - using the same naming convention as in ThreeDeadWheelLocalizer
        // Adjust the motor names if needed to match your configuration
        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));
        backEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        
        // Set encoder directions if needed
        // Example: leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Reset PID timer
        pidTimer.reset();
    }
    
    /**
     * Set the target position for the robot
     * @param targetPose The target pose (x, y, heading)
     */
    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }
    
    /**
     * Get the current position of the robot based on encoder readings
     * @return Current pose (x, y, heading)
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }
    
    /**
     * Update the current position based on encoder readings
     */
    public void updatePosition() {
        // Get encoder positions and velocities
        PositionVelocityPair leftPosVel = leftEncoder.getPositionAndVelocity();
        PositionVelocityPair rightPosVel = rightEncoder.getPositionAndVelocity();
        PositionVelocityPair backPosVel = backEncoder.getPositionAndVelocity();
        
        // Convert encoder counts to inches
        double leftPos = leftPosVel.position * inchesPerCount;
        double rightPos = rightPosVel.position * inchesPerCount;
        double backPos = backPosVel.position * inchesPerCount;
        
        // Calculate position and heading
        double heading = (rightPos - leftPos) / encoderTrackWidth;
        double x = backPos;
        double y = (leftPos + rightPos) / 2.0;
        
        // Update current pose
        currentPose = new Pose2d(x, y, heading);
    }
    
    /**
     * Calculate PID output for robot control
     * @return PoseVelocity2d containing the velocity commands for the robot
     */
    public PoseVelocity2d calculatePIDOutput() {
        // Update current position
        updatePosition();
        
        // Calculate errors
        xError = targetPose.position.x - currentPose.position.x;
        yError = targetPose.position.y - currentPose.position.y;
        headingError = targetPose.heading.minus(currentPose.heading).toDouble();
        
        // Calculate time delta
        double dt = pidTimer.seconds();
        pidTimer.reset();
        
        // Prevent division by zero
        if (dt < 1e-6) {
            dt = 1e-6;
        }
        
        // Calculate integral terms with anti-windup
        xIntegralSum += xError * dt;
        yIntegralSum += yError * dt;
        headingIntegralSum += headingError * dt;
        
        // Apply integral limits to prevent windup
        xIntegralSum = Math.max(-X_PID_COEFFICIENTS.integralSumLimit, Math.min(xIntegralSum, X_PID_COEFFICIENTS.integralSumLimit));
        yIntegralSum = Math.max(-Y_PID_COEFFICIENTS.integralSumLimit, Math.min(yIntegralSum, Y_PID_COEFFICIENTS.integralSumLimit));
        headingIntegralSum = Math.max(-HEADING_PID_COEFFICIENTS.integralSumLimit, Math.min(headingIntegralSum, HEADING_PID_COEFFICIENTS.integralSumLimit));
        
        // Calculate derivative terms
        double xDerivative = (xError - lastXError) / dt;
        double yDerivative = (yError - lastYError) / dt;
        double headingDerivative = (headingError - lastHeadingError) / dt;
        
        // Save current errors for next iteration
        lastXError = xError;
        lastYError = yError;
        lastHeadingError = headingError;
        
        // Calculate PID outputs
        double xOutput = X_PID_COEFFICIENTS.kP * xError + 
                        X_PID_COEFFICIENTS.kI * xIntegralSum + 
                        X_PID_COEFFICIENTS.kD * xDerivative;
        
        double yOutput = Y_PID_COEFFICIENTS.kP * yError + 
                        Y_PID_COEFFICIENTS.kI * yIntegralSum + 
                        Y_PID_COEFFICIENTS.kD * yDerivative;
        
        double headingOutput = HEADING_PID_COEFFICIENTS.kP * headingError + 
                              HEADING_PID_COEFFICIENTS.kI * headingIntegralSum + 
                              HEADING_PID_COEFFICIENTS.kD * headingDerivative;
        
        // Return velocity commands
        return new PoseVelocity2d(new Vector2d(xOutput, yOutput), headingOutput);
    }
    
    /**
     * Reset the PID controller
     */
    public void reset() {
        xError = 0.0;
        yError = 0.0;
        headingError = 0.0;
        
        xIntegralSum = 0.0;
        yIntegralSum = 0.0;
        headingIntegralSum = 0.0;
        
        lastXError = 0.0;
        lastYError = 0.0;
        lastHeadingError = 0.0;
        
        pidTimer.reset();
    }
}