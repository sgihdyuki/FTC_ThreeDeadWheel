package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * EncoderPIDControllerDemo - A demo OpMode that shows how to use the EncoderPIDController
 * to control a robot with three dead wheel encoders (two for y-axis, one for x-axis)
 */
@Config
@TeleOp(name = "Encoder PID Controller Demo", group = "Demo")
public class EncoderPIDControllerDemo extends LinearOpMode {
    // Configuration parameters
    public static double INCHES_PER_COUNT = 0.0007639; // Adjust based on your encoder and wheel specs
    public static double ENCODER_TRACK_WIDTH = 10.0; // Distance between left and right encoders in inches
    
    // Motor power limits
    public static double MAX_POWER = 0.8;
    
    // Drive motors
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightBackMotor;
    
    // PID Controller
    private EncoderPIDController pidController;
    
    // Target pose
    private Pose2d targetPose = new Pose2d(0, 0, 0);
    
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
        
        // Initialize PID controller
        pidController = new EncoderPIDController(hardwareMap, INCHES_PER_COUNT, ENCODER_TRACK_WIDTH);
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        // Reset the PID controller
        pidController.reset();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Update current position from encoders
            pidController.updatePosition();
            Pose2d currentPose = pidController.getCurrentPose();
            
            // Handle gamepad input to set target position
            handleGamepadInput();
            
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
            telemetry.update();
        }
        
        // Stop all motors
        setAllMotorPowers(0);
    }
    
    /**
     * Handle gamepad input to set target position
     */
    private void handleGamepadInput() {
        // Get current target
        double targetX = targetPose.position.x;
        double targetY = targetPose.position.y;
        double targetHeading = targetPose.heading.toDouble();
        
        // Gamepad 1 left stick controls target X/Y position
        if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
            // Scale the input for fine control (adjust the multiplier as needed)
            targetX += -gamepad1.left_stick_y * 0.5; // Inverted Y axis
            targetY += -gamepad1.left_stick_x * 0.5; // Inverted X axis for strafing
        }
        
        // Gamepad 1 right stick controls target heading
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            // Scale the input for fine control
            targetHeading += gamepad1.right_stick_x * 0.05;
        }
        
        // Reset target position when B is pressed
        if (gamepad1.b) {
            targetX = 0;
            targetY = 0;
            targetHeading = 0;
            pidController.reset();
        }
        
        // Update target pose
        targetPose = new Pose2d(targetX, targetY, targetHeading);
        pidController.setTargetPose(targetPose);
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