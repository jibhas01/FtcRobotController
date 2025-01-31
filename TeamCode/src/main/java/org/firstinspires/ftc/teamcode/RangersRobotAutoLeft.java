package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RangersRobotAutoLeft")
public class RangersRobotAutoLeft  extends LinearOpMode {
    // Motors
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    DcMotorEx arm;
    Servo wrist;
    Servo claw;
    IMU imu;

    // PID
    double integralSum = 0;
    static double kp = 0.03;
    static double ki = 0;
    static double kd = 0.0003;
    static double f = 0;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    final double ticks_in_degrees = 537.7 / 360;

    // Wheels
    double wheelTarget1 = 300;
    double wheelTarget2 = 300;
    double wheelTarget3 = 300;
    double wheelTarget4 = -200;
    double robotTolerance = 0.1;
    boolean doneRobotMove1 = false;
    boolean doneRobotMove2 = true;
    boolean doneRobotMove3 = true;
    boolean doneRobotMove4 = true;

    // Arm
    final double minArmPos = 10;
    final double maxArmPos = 1765;
    double initialArmPos = 0;
    double armSpeedInc = 2;
    double armTarget1 = 300;
    double armTarget2 = 400;
    double armTolerance = 0.1;
    boolean doneArmMove1 = true;
    boolean doneArmMove2 = true;
    boolean doneArmMove3 = true;

    // Wrist
    double initialWristPos = 0.15;
    double maxWristPos = 0.7;
    double minWristPos = 0;
    double wristSpeedInc = 0.004;
    double wristTarget1 = 0.3;
    double wristTarget2 = 0.5;
    boolean doneWristMove1 = true;
    boolean doneWristMove2 = true;

    // Claw
    double initialClawPos = 0.75;
    double maxClawPos = 0.78;
    double minClawPos = 0.65;
    double clawSpeedInc = 0.004;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        lf = initDcMotor(hardwareMap, "fl", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "fr", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "bl", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "br", DcMotor.Direction.FORWARD);
        arm = initDcMotor(hardwareMap, "arm", DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(params);

        arm.setPower(initialArmPos);
        wrist.setPosition(initialWristPos);
        claw.setPosition(initialClawPos);

        waitForStart();

        while (opModeIsActive()) {
            double lfPos = lf.getCurrentPosition();
            // 1st robot move
            if(!doneRobotMove1) {
                driveXYW(pidController(lfPos + wheelTarget1, lfPos), 0, 0);
                if(Math.abs(wheelTarget1 - lf.getCurrentPosition()) < robotTolerance){
                    doneRobotMove1 = true;
                    doneRobotMove2 = false;
                }
            }

            // 2nd robot move
            if(!doneRobotMove2) {
                driveXYW(0, pidController(lfPos + wheelTarget2, lfPos), 0);
                if(Math.abs(wheelTarget2 - lf.getCurrentPosition()) < robotTolerance){
                    doneRobotMove2 = true;
                    doneRobotMove3 = false;
                    doneArmMove1 = false;
                    doneWristMove1 = false;
                }
            }

            // 3rd robot move
            if(!doneRobotMove3) {
                driveXYW(pidController(lfPos + wheelTarget3, lfPos), 0, 0);
                if(Math.abs(wheelTarget3 - lf.getCurrentPosition()) < 0.1){
                    doneRobotMove3 = true;
                    doneArmMove2 = false;
                    doneWristMove2 = false;
                }
            }

            // 4th robot move
            if(!doneRobotMove4) {
                driveXYW(pidController(lfPos + wheelTarget4, lfPos), 0, 0);
                if(Math.abs(wheelTarget4 - lf.getCurrentPosition()) < 0.1){
                    doneRobotMove4 = true;
                }
            }

            // 1st arm move
            if(!doneArmMove1){
                arm.setPower(pidController(armTarget1, arm.getCurrentPosition()));
                if(Math.abs(armTarget1 - arm.getCurrentPosition()) < armTolerance){
                    doneArmMove1 = true;
                }
            }

            // 2nd arm move
            if(!doneArmMove2){
                arm.setPower(pidController(armTarget2, arm.getCurrentPosition()));
                if(Math.abs(armTarget2 - arm.getCurrentPosition()) < armTolerance){
                    doneArmMove2 = true;
                }
            }

            // 1st wrist move
            if(!doneWristMove1){
                wrist.setPosition(wristTarget1);
                doneWristMove1 = true;
            }

            // 2nd wrist move
            if(!doneWristMove2){
                wrist.setPosition(wristTarget2);
                doneWristMove2 = true;
            }

        }
    }

    public DcMotorEx initDcMotor(HardwareMap hardwareMap,
                                 String name,
                                 DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return m;
    }

    public void driveXYW(double rx, double ry, double rw) {
        double lfPower = rx - ry - rw;
        double rfPower = rx + ry + rw;
        double lbPower = rx + ry - rw;
        double rbPower = rx - ry + rw;

        double maxPower = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower),
                        Math.max(Math.abs(lbPower), Math.abs(rbPower))));

        if(maxPower > 1){
            lfPower /= maxPower;
            rfPower /= maxPower;
            lbPower /= maxPower;
            rbPower /= maxPower;
        }

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
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
}
