package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ThreeWheelEncoderAutonomous - An autonomous OpMode that uses three dead wheel encoders
 * for precise positioning and movement control.
 */
@Config
@Autonomous(name = "Three Wheel Encoder Autonomous", group = "Autonomous")
public class ThreeWheelEncoderAutonomous extends LinearOpMode {
    // Configuration parameters
    public static double INCHES_PER_COUNT = 0.0007639; // Adjust based on your encoder and wheel specs
    public static double ENCODER_TRACK_WIDTH = 10.0; // Distance between left and right encoders in inches
    
    // PID tuning parameters
    public static double X_P = 0.05;
    public static double X_I = 0.0;
    public static double X_D = 0.001;
    
    public static double Y_P = 0.05;
    public static double Y_I = 0.0;
    public static double Y_D = 0.001;
    
    public static double HEADING_P = 0.05;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.001;
    
    // Position tolerance for considering a move complete
    public static double POSITION_TOLERANCE = 0.5; // inches
    public static double HEADING_TOLERANCE = Math.toRadians(2.0); // radians
    
    // Motor power limits
    public static double MAX_POWER = 0.8;
    
    // Timeout for movements
    public static double MOVEMENT_TIMEOUT = 5.0; // seconds
    
    // Drive motors
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightBackMotor;
    
    // PID Controller
    private EncoderPIDController pidController;
    
    // Timer
    private ElapsedTime moveTimer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        // Set up telemetry dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize drive motors
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Set motor directions
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set zero power behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize PID controller with custom coefficients
        pidController = new EncoderPIDController(hardwareMap, INCHES_PER_COUNT, ENCODER_TRACK_WIDTH);
        
        // Update PID coefficients
        EncoderPIDController.X_PID_COEFFICIENTS = new EncoderPIDController.PIDCoefficients(X_P, X_I, X_D, 0.5);
        EncoderPIDController.Y_PID_COEFFICIENTS = new EncoderPIDController.PIDCoefficients(Y_P, Y_I, Y_D, 0.5);
        EncoderPIDController.HEADING_PID_COEFFICIENTS = new EncoderPIDController.PIDCoefficients(HEADING_P, HEADING_I, HEADING_D, 0.5);
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        // Reset the PID controller
        pidController.reset();
        
        // Execute autonomous path
        if (opModeIsActive()) {
            // Example autonomous sequence
            // Move forward 24 inches
            moveToPosition(new Pose2d(0, 24, 0));
            
            // Turn 90 degrees
            moveToPosition(new Pose2d(0, 24, Math.toRadians(90)));
            
            // Move to position (24, 24) while maintaining heading
            moveToPosition(new Pose2d(24, 24, Math.toRadians(90)));
            
            // Return to starting position
            moveToPosition(new Pose2d(0, 0, 0));
        }
        
        // Stop all motors
        setAllMotorPowers(0);
    }
    
    /**
     * Move the robot to a specific position using PID control
     * @param targetPose The target position and heading
     */
    private void moveToPosition(Pose2d targetPose) {
        // Set the target pose for the PID controller
        pidController.setTargetPose(targetPose);
        
        // Reset the move timer
        moveTimer.reset();
        
        telemetry.addData("Status", "Moving to position: X=%.2f, Y=%.2f, Heading=%.2f",
                targetPose.position.x, targetPose.position.y, Math.toDegrees(targetPose.heading.toDouble()));
        telemetry.update();
        
        // Continue until we reach the target position or timeout
        while (opModeIsActive() && !isPositionReached(targetPose) && moveTimer.seconds() < MOVEMENT_TIMEOUT) {
            // Update current position from encoders
            pidController.updatePosition();
            Pose2d currentPose = pidController.getCurrentPose();
            
            // Calculate PID output
            PoseVelocity2d pidOutput = pidController.calculatePIDOutput();
            
            // Apply mecanum drive kinematics to convert to wheel powers
            double[] wheelPowers = calculateMecanumWheelPowers(pidOutput);
            
            // Set motor powers
            leftFrontMotor.setPower(wheelPowers[0]);
            leftBackMotor.setPower(wheelPowers[1]);
            rightFrontMotor.setPower(wheelPowers[2]);
            rightBackMotor.setPower(wheelPowers[3]);
            
            // Display telemetry
            telemetry.addData("Target X", "%.2f", targetPose.position.x);
            telemetry.addData("Target Y", "%.2f", targetPose.position.y);
            telemetry.addData("Target Heading", "%.2f", Math.toDegrees(targetPose.heading.toDouble()));
            telemetry.addData("Current X", "%.2f", currentPose.position.x);
            telemetry.addData("Current Y", "%.2f", currentPose.position.y);
            telemetry.addData("Current Heading", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("X Error", "%.2f", targetPose.position.x - currentPose.position.x);
            telemetry.addData("Y Error", "%.2f", targetPose.position.y - currentPose.position.y);
            telemetry.addData("Heading Error", "%.2f", Math.toDegrees(targetPose.heading.minus(currentPose.heading).toDouble()));
            telemetry.addData("Move Timer", "%.1f", moveTimer.seconds());
            telemetry.update();
        }
        
        // Stop motors after reaching position or timeout
        setAllMotorPowers(0);
        
        // Check if we reached the target or timed out
        if (isPositionReached(targetPose)) {
            telemetry.addData("Status", "Position reached");
        } else {
            telemetry.addData("Status", "Movement timed out");
        }
        telemetry.update();
        
        // Small delay to stabilize
        sleep(200);
    }
    
    /**
     * Check if the robot has reached the target position within tolerance
     * @param targetPose The target position to check against
     * @return True if position is reached, false otherwise
     */
    private boolean isPositionReached(Pose2d targetPose) {
        Pose2d currentPose = pidController.getCurrentPose();
        
        // Calculate position and heading errors
        double xError = Math.abs(targetPose.position.x - currentPose.position.x);
        double yError = Math.abs(targetPose.position.y - currentPose.position.y);
        double headingError = Math.abs(targetPose.heading.minus(currentPose.heading).toDouble());
        
        // Check if errors are within tolerance
        return xError < POSITION_TOLERANCE && 
               yError < POSITION_TOLERANCE && 
               headingError < HEADING_TOLERANCE;
    }
    
    /**
     * Calculate mecanum wheel powers from velocity commands
     * @param velocity The desired robot velocity
     * @return Array of wheel powers [leftFront, leftBack, rightFront, rightBack]
     */
    private double[] calculateMecanumWheelPowers(PoseVelocity2d velocity) {
        double x = velocity.linearVel.x;
        double y = velocity.linearVel.y;
        double omega = velocity.angVel;
        
        // Calculate wheel powers using mecanum kinematics
        double leftFrontPower = y + x + omega;
        double leftBackPower = y - x + omega;
        double rightFrontPower = y - x - omega;
        double rightBackPower = y + x - omega;
        
        // Normalize wheel powers
        double maxPower = Math.max(
            Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
            Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
        );
        
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }
        
        // Apply power limit
        leftFrontPower *= MAX_POWER;
        leftBackPower *= MAX_POWER;
        rightFrontPower *= MAX_POWER;
        rightBackPower *= MAX_POWER;
        
        return new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};
    }
    
    /**
     * Set all motor powers to the same value
     * @param power The power to set
     */
    private void setAllMotorPowers(double power) {
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }
}