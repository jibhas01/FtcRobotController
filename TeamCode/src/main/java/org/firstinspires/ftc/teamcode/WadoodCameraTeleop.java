package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "WadoodCameraTeleop", group = "Linear Opmode")
public class WadoodCameraTeleop extends LinearOpMode {

    // Drive motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    // Shooter motors
    private DcMotorEx outRight;
    private DcMotorEx outLeft;

    // Intake motor
    private DcMotor intake;



    // Shooter config (base values)
    private static final double DEFAULT_SHOOT_POWER = -0.9; // fallback when no tag
    private static final double FEED_POWER  = 0.6;          // intake feed power
    private static final int    FEED_TICKS  = 1500;         // ticks before intake starts feeding

    // Shooter state
    private boolean previousA = false;
    private boolean shootingActive = false;
    private boolean feeding = false;
    private int startRightTicks = 0;
    private int startLeftTicks  = 0;
    private double currentShootPower = DEFAULT_SHOOT_POWER;



    // ---- VISION / APRILTAG ----
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double lastRangeInches = -1;
    private double lastBearingDeg  = 0;

    // Camera yaw offset: set to 0 so we align to actual tag center
    private static final double CAMERA_YAW_OFFSET_DEG = 0.0;

    @Override
    public void runOpMode() {

        // -------- HARDWARE MAP --------
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");

        outRight   = hardwareMap.get(DcMotorEx.class, "outRight");
        outLeft    = hardwareMap.get(DcMotorEx.class, "outLeft");

        intake     = hardwareMap.get(DcMotor.class, "intake");



        // Drive directions (typical mecanum; flip if needed)
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Shooter directions (one reversed so wheels spin same way)
        outRight.setDirection(DcMotor.Direction.FORWARD);
        outLeft.setDirection(DcMotor.Direction.REVERSE);

        // Zero power behavior
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ----- DRIVE ENCODERS -----
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ----- SHOOTER ENCODERS -----
        outRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ----- VISION / APRILTAG SETUP -----
        initAprilTag();

        telemetry.addLine("Wadood TeleOp + AprilTag ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --------- UPDATE APRILTAG INFO ---------
            updateAprilTagData();

            // ------------------------------
            // DRIVE CONTROL (gamepad1)
            // ------------------------------
            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  =  gamepad1.left_stick_x;   // strafe
            double rx = -gamepad1.right_stick_x;  // rotate

            // Deadzone
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            // Auto-align using LEFT BUMPER + AprilTag
            if (gamepad1.left_bumper && lastRangeInches > 0) {
                // We want bearing -> 0. Use error = bearing + any camera offset (currently 0)
                double errorDeg = lastBearingDeg + CAMERA_YAW_OFFSET_DEG;

                // Small deadzone to avoid jitter when almost centered
                if (Math.abs(errorDeg) < 0.5) {
                    rx = 0.0;
                } else {
                    double kP = 0.02; // tune if too slow/fast
                    // If this turns the wrong direction, flip the sign on this line:
                    rx = errorDeg * kP;

                    // Clamp turn power
                    if (rx > 0.4) rx = 0.4;
                    if (rx < -0.4) rx = -0.4;
                }
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            // ------------------------------
            // SHOOTER CONTROL (hold A, use distance-based power)
            // ------------------------------
            boolean currentA = gamepad1.a;

            if (currentA) {
                // When A is first pressed, start a new shot
                if (!previousA) {
                    shootingActive = true;
                    feeding = false;

                    // Record starting encoder positions
                    startRightTicks = outRight.getCurrentPosition();
                    startLeftTicks  = outLeft.getCurrentPosition();

                    // Pick shooter power using AprilTag distance (if we have one)
                    currentShootPower = getShooterPowerForRange(lastRangeInches);

                    outRight.setPower(currentShootPower);
                    outLeft.setPower(currentShootPower);
                }

                if (shootingActive) {
                    int deltaRight = Math.abs(outRight.getCurrentPosition() - startRightTicks);
                    int deltaLeft  = Math.abs(outLeft.getCurrentPosition()  - startLeftTicks);

                    // Once both motors have moved at least FEED_TICKS, start feeding
                    if (!feeding && deltaRight >= FEED_TICKS && deltaLeft >= FEED_TICKS) {
                        feeding = true;
                    }
                }

            } else {
                // A released: stop shooters and feeding
                shootingActive = false;
                feeding = false;

                outRight.setPower(0);
                outLeft.setPower(0);
            }

            previousA = currentA;

            // ------------------------------
            // INTAKE CONTROL
            // ------------------------------
            double intakePower;

            if (feeding) {
                // Auto-feed into shooter once shooters reached FEED_TICKS
                intakePower = FEED_POWER;
            } else {
                // Manual control with triggers when not feeding
                if (gamepad1.right_trigger > 0.1) {
                    intakePower = gamepad1.right_trigger;         // intake in
                } else if (gamepad1.left_trigger > 0.1) {
                    intakePower = -gamepad1.left_trigger;         // outtake
                } else {
                    intakePower = 0.0;
                }
            }

            intake.setPower(intakePower);

            // ------------------------------
            // TELEMETRY
            // ------------------------------
            telemetry.addData("Drive Power", "FL: %.2f FR: %.2f BL: %.2f BR: %.2f",
                    flPower, frPower, blPower, brPower);
            telemetry.addData("FR Enc", frontRight.getCurrentPosition());
            telemetry.addData("FL Enc", frontLeft.getCurrentPosition());
            telemetry.addData("BR Enc", backRight.getCurrentPosition());
            telemetry.addData("BL Enc", backLeft.getCurrentPosition());

            telemetry.addData("ShooterActive", shootingActive);
            telemetry.addData("Feeding", feeding);
            telemetry.addData("outRight pos", outRight.getCurrentPosition());
            telemetry.addData("outLeft pos", outLeft.getCurrentPosition());
            telemetry.addData("ShootPower", "%.2f", currentShootPower);

            telemetry.addData("Tag Range (in)", "%.1f", lastRangeInches);
            telemetry.addData("Tag Bearing (deg)", "%.1f", lastBearingDeg);
            telemetry.addData("Error Used (deg)", "%.1f", lastBearingDeg + CAMERA_YAW_OFFSET_DEG);

            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.update();
        }
    }

    // ================= APRILTAG SETUP & UPDATE =================

    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        // Use Logitech Webcam named "Webcam 1" in config
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTag)
                .build();
    }

    private void updateAprilTagData() {
        lastRangeInches = -1;
        lastBearingDeg  = 0;

        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) {
            return;
        }

        // Choose the detection whose center is closest to the middle (smallest |bearing|)
        AprilTagDetection bestTag = null;
        double bestAbsBearing = Double.MAX_VALUE;

        for (AprilTagDetection det : detections) {
            if (det.ftcPose != null) {
                double absBearing = Math.abs(det.ftcPose.bearing);
                if (absBearing < bestAbsBearing) {
                    bestAbsBearing = absBearing;
                    bestTag = det;
                }
            }
        }

        if (bestTag != null && bestTag.ftcPose != null) {
            lastRangeInches = bestTag.ftcPose.range;   // distance in inches
            lastBearingDeg  = bestTag.ftcPose.bearing; // angle offset from camera center in degrees
        }
    }

    // ================= SHOOTER POWER FROM RANGE =================

    /**
     * Pick shooter power based on AprilTag distance.
     * You SHOULD tune these numbers on the field.
     */
    private double getShooterPowerForRange(double rangeInches) {
        // If no tag seen, just use default
        if (rangeInches <= 0 || Double.isNaN(rangeInches)) {
            return DEFAULT_SHOOT_POWER;
        }

        // Example curve – TUNE THESE:
        double power;
        if (rangeInches <= 24) {
            power = -0.70;
        } else if (rangeInches <= 36) {
            power = -0.80;
        } else if (rangeInches <= 48) {
            power = -0.85;
        } else {
            power = -0.90;
        }

        return power;
    }
}
