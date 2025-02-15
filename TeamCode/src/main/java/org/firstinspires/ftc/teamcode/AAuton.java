package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Power Auton", group="Linear OpMode")
public class AAuton extends LinearOpMode {

    // Motors
    DcMotorEx lf, rf, lb, rb, arm;
    Servo wrist, claw;
    IMU imu;

    // Constants
    final double ticks_in_degrees = 537.7 / 360;
    final double armHoldPower = 0.2; // Adjust to prevent arm from falling

    @Override
    public void runOpMode() {

        // Initialization
        lf = initDcMotor("fl", DcMotor.Direction.REVERSE);
        rf = initDcMotor("fr", DcMotor.Direction.FORWARD);
        lb = initDcMotor("bl", DcMotor.Direction.REVERSE);
        rb = initDcMotor("br", DcMotor.Direction.FORWARD);
        arm = initDcMotor("arm", DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set initial positions
        wrist.setPosition(0);
        claw.setPosition(0.85);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Raise arm to 400 and hold it
            moveRobot(-100, 0.5);
            moveArmToPosition(490);

            // Set wrist to 0.5
            wrist.setPosition(0.5);
            sleep(500);

            // Move forward 800 ticks
            moveRobot(-550, 0.5);

            // Lower arm to 300
            moveArmToPosition(300);

            // Reverse 200 ticks
            moveRobot(200, 0.5);

            // Open claw
            claw.setPosition(0.35);
            sleep(500);

            // Reverse 200 ticks
            moveRobot(200, 0.5);

            // Strafe right 500 ticks
            // strafeRobot(1350, 0.5);
            // sleep(500);
            // moveRobot(-1200, 0.5);
            // sleep(500);
            // strafeRobot(600, 0.5);
            // sleep(500);
            // moveRobot(1500, 0.5);
            // sleep(500);
            // moveRobot(-1100, 0.5);
            // sleep(500);
            // strafeRobot(500, 0.5);
            // sleep(500);
            // moveRobot(1500, 0.5);
        }
    }

    private DcMotorEx initDcMotor(String name, DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    private void moveArmToPosition(int targetPosition) {
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(armHoldPower); // Hold arm position
    }

    private void moveRobot(int distance, double power) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setTargetPosition(distance);
        rf.setTargetPosition(distance);
        lb.setTargetPosition(distance);
        rb.setTargetPosition(distance);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(power);
        rf.setPower(power);
        lb.setPower(power);
        rb.setPower(power);

        while (opModeIsActive() && lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy()) {
            telemetry.addData("Moving", "Distance: %d", distance);
            telemetry.update();
        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    private void strafeRobot(int distance, double power) {
        lf.setTargetPosition(-distance);
        rf.setTargetPosition(distance);
        lb.setTargetPosition(distance);
        rb.setTargetPosition(-distance);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(power);
        rf.setPower(power);
        lb.setPower(power);
        rb.setPower(power);

        while (opModeIsActive() && lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy()) {
            telemetry.addData("Strafing", "Distance: %d", distance);
            telemetry.update();
        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
