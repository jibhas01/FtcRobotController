package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp(name="Rangers Robot", group="Linear OpMode")

public class RangersRobot extends LinearOpMode {

    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    DcMotorEx arm;
    Servo wrist;
    Servo claw;
    IMU imu;

    static double headingOffset = 0;
    double integralSum = 0;
    static double kp = 0.02;
    static double ki = 0.002;
    static double kd = 0.0002;
    static double f = 0.5;
    double armTarget = 0;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    final double ticks_in_degrees = 537.7 / 360;
    final double minArmPos = 0;
    final double maxArmPos = 377;
    double speedIncrementer = 0;

    @Override
    public void runOpMode() {
        init(hardwareMap);

        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            driveFieldXYW(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            moveArm(-gamepad2.left_stick_y, gamepad2.right_stick_y, gamepad2.left_trigger, gamepad2.right_trigger);

            //telemetry.addData("gamepad1.y: ", gamepad1.y);
            //telemetry.addData("previousGamePad1.y" , previousGamepad1.y);

            if (gamepad1.y && previousGamepad1.y) {
                setHeading(0);
                telemetry.addData("Status" , "Reset Heading");
                previousGamepad1.copy(gamepad1);
            }

//            if (!gamepad1.y && previousGamepad1.y) {
//                //does nothing
//            }

            previousGamepad1 = gamepad1;

            telemetry.addData("Status", "Running");
            telemetry.addData("IMU heading", getIMUHeading());
            telemetry.addData("wrist pos: ", wrist.getPosition());
            telemetry.addData("claw pos: ", claw.getPosition());
            telemetry.update();
        }
    }

    /*
     * a: to move the arm
     * w: to more the wrist
     * oc, cc: to open claw or close claw
     */
    private void moveArm(double a, double w, float oc, float cc) {
        int armPos = arm.getCurrentPosition();
        if (a > 0) {
            armTarget = armTarget + a + speedIncrementer;
        }else if(a < 0) {
            armTarget = armTarget - a - speedIncrementer;
        }

        if (armTarget > maxArmPos){
            armTarget = maxArmPos;
        }
        if (armTarget < minArmPos){
            armTarget = minArmPos;
        }
        telemetry.addData("a: ", a);
        telemetry.addData("w: ", w);
        telemetry.addData("oc: ", oc);
        telemetry.addData("cc: ", cc);

        arm.setPower(pidController(armTarget, armPos));

        // Add code to move the wrist and claw using w, cl, and cr
    }

    public void init(HardwareMap hardwareMap) {
        lf = initDcMotor(hardwareMap, "fl", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "fr", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "bl", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "br", DcMotor.Direction.FORWARD);
        arm = initDcMotor(hardwareMap, "arm", DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        initIMU(hardwareMap);
    }

    public DcMotorEx initDcMotor(HardwareMap hardwareMap,
                                 String name,
                                 DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    public void driveXYW(double rx, double ry, double rw) {
        double lfPower = rx - ry - rw;
        double rfPower = rx + ry + rw;
        double lbPower = rx + ry - rw;
        double rbPower = rx - ry + rw;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    public void driveFieldXYW(double fx, double fy, double fw) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading());
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
    }

    public double pidController(double target, double state){
        double error = target - state;
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;
        elapsedTime.reset();
        double pid = (error * kp) + (derivative * kd) + (integralSum * ki);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        telemetry.addData("pid: ", pid);
        telemetry.addData("ff: ", ff);
        telemetry.addData("target: ", target);
        telemetry.addData("armPos", state);

        return pid + ff;
    }

    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(params);
    }

    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setHeading(double h) {
        headingOffset = h - getIMUHeading();
        imu.resetYaw();
    }

    public double getHeading() {
        return AngleUnit.normalizeDegrees(headingOffset + getIMUHeading());
    }

}
