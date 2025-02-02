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

@Autonomous(name = "RangersRobotAutoRight")
public class RangersRobotAutoRight  extends LinearOpMode {
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
    static double kp = 0.017;
    static double ki = 0;
    static double kd = 0;
    static double f = 0;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    final double ticks_in_degrees = 537.7 / 360;

    // Wheels
    int wheelTarget1 = -50;
    int wheelTarget2 = 1200;
    int wheelTarget3 = -200;
    int wheelTarget4 = 150;
    double robotTolerance = 2;
    boolean doneRobotMove1 = false;
    boolean doneRobotMove2 = true;
    boolean doneRobotMove3 = true;
    boolean doneRobotMove4 = true;

    // Arm
    final double minArmPos = 10;
    final double maxArmPos = 1765;
    double initialArmPos = 20;
    double armSpeedInc = 4;
    int armTarget1 = 460;
    int armTarget2 = 15;
    double armTolerance = 2;
    boolean doneArmMove1 = true;
    boolean doneArmMove2 = true;
    boolean doneArmMove3 = true;

    // Wrist
    double initialWristPos = 0;
    double maxWristPos = 0.7;
    double minWristPos = 0;
    double wristSpeedInc = 0.004;
    double wristTarget1 = 0.37;
    double wristTarget2 = 0.4;
    double wristTarget3 = 0.25;
    boolean doneWristMove1 = true;
    boolean doneWristMove2 = true;
    boolean doneWristMove3 = true;

    // Claw
    double initialClawPos = 0.77;
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
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(params);

        //arm.setPower(initialArmPos);
        wrist.setPosition(initialWristPos);
        claw.setPosition(initialClawPos);
        double lfLastPos = lf.getCurrentPosition();
        double interimArmTarget = 0;

        // telemetry.addData("lfLastPos: ", lfLastPos);
        // telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lfCurrPos = lf.getCurrentPosition();
            double rfCurrPos = rf.getCurrentPosition();
            double lbCurrPos = lb.getCurrentPosition();
            double rbCurrPos = rb.getCurrentPosition();
            // 1st robot move
            if(!doneRobotMove1) {
                driveWithEncoders(wheelTarget1, 0, 0);
                //if(Math.abs(Math.abs(wheelTarget1) - Math.abs(lfCurrPos)) < robotTolerance){
                doneRobotMove1 = true;
                doneRobotMove2 = false;
                lfLastPos = lfCurrPos;
                //}
            }

            // 2nd robot move
            if(!doneRobotMove2) {
                driveWithEncoders(0, wheelTarget2, 0);
                //if(Math.abs(Math.abs(wheelTarget2) - Math.abs(lfCurrPos)) < robotTolerance){
                doneRobotMove2 = true;
                doneRobotMove3 = false;
                doneArmMove1 = false;
                //}
            }

            // 3rd robot move
//            if(!doneRobotMove3) {
//                //driveWithEncoders(wheelTarget3, 0, 0);
//                //if(Math.abs(Math.abs(wheelTarget3) - Math.abs(lfCurrPos)) < robotTolerance){
//                    doneRobotMove3 = true;
//                    doneWristMove2 = false;
//                //}
//            }
//
//            // 4th robot move
//            if(!doneRobotMove4) {
//                driveWithEncoders(wheelTarget4, 0, 0);
//                if(Math.abs(Math.abs(wheelTarget4) - Math.abs(lfCurrPos)) < robotTolerance){
//                    doneRobotMove4 = true;
//                    doneWristMove3 = false;
//                }
//            }
//
//            // 1st arm move
            //moveArmWithPID(initialArmPos);
//            if(!doneArmMove1){
//                //initialArmPos = armTarget1;
//                //moveArmWithEncoders(armTarget1);
//                //moveArmWithPID(armTarget1);
//                interimArmTarget += armSpeedInc;
//                if(interimArmTarget > armTarget1){
//                    doneArmMove1 = true;
//                    doneWristMove1 = false;
//                }
//            }
//            arm.setPower(pidController(interimArmTarget, arm.getCurrentPosition()));
//
//            // 2nd arm move
//            if(!doneArmMove2){
//                //initialArmPos = armTarget2;
//                moveArmWithEncoders(armTarget2);
//                //moveArmWithPID(armTarget2);
//                if(Math.abs(armTarget2 - arm.getCurrentPosition()) < armTolerance){
//                    doneArmMove2 = true;
//                }
//            }
//
//            // 1st wrist move
//            if(!doneWristMove1){
//                wrist.setPosition(wristTarget1);
//                doneWristMove1 = true;
//            }
//
//            // 2nd wrist move
//            if(!doneWristMove2){
//                wrist.setPosition(wristTarget2);
//                doneWristMove2 = true;
//                doneRobotMove4 = false;
//            }
//
//            // 3rd wrist move
//            if(!doneWristMove3){
//                wrist.setPosition(wristTarget3);
//                doneWristMove3 = true;
//                doneArmMove2 = false;
//            }

//            telemetry.addData("lf pos: ", lfCurrPos);
//            telemetry.addData("rf pos: ", rfCurrPos);
//            telemetry.addData("lb pos: ", lbCurrPos);
//            telemetry.addData("rb pos: ", rbCurrPos);
//           telemetry.addData("tolerance: ", Math.abs(wheelTarget1 - lfCurrPos));
//            telemetry.addData("armTolerance: ", Math.abs(armTarget2 - arm.getCurrentPosition()));
            telemetry.addData("doneRobotMove1: ", doneRobotMove1);
            telemetry.addData("doneRobotMove2: ", doneRobotMove2);
            telemetry.addData("doneRobotMove3: ", doneRobotMove3);
            telemetry.addData("doneRobotMove4: ", doneRobotMove4);
            telemetry.addData("doneArmMove1: ", doneArmMove1);
            telemetry.addData("doneWristMove1: ", doneWristMove1);
            telemetry.update();
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
        telemetry.addData("currPos: ", state);

        return pid + ff;
    }

    public void driveWithEncoders(int rx, int ry, int rw){
        lf.setTargetPosition(lf.getCurrentPosition() + rx - ry - rw);
        rf.setTargetPosition(rf.getCurrentPosition() + rx + ry + rw);
        lb.setTargetPosition(lb.getCurrentPosition() + rx + ry - rw);
        rb.setTargetPosition(rb.getCurrentPosition() + rx - ry + rw);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(0.2);
        lb.setPower(0.2);
        rf.setPower(0.2);
        rb.setPower(0.2);

        while (opModeIsActive() && (lf.isBusy() || lb.isBusy() || rf.isBusy() || rb.isBusy())) {
            idle();
        }
//        try {
//            Thread.sleep(5000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
    }

    public void moveArmWithEncoders(int target){
        arm.setTargetPosition(target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.3);

        while (opModeIsActive() && arm.isBusy()) {
            idle();
        }
    }

//    public double moveArmWithPID(double target){
//        double currentPos = arm.getCurrentPosition();
//        double myTarget = 0;
//        if (target > currentPos) {
//            myTarget = currentPos + armSpeedInc;
//        }
//        if(target < currentPos) {
//            myTarget = currentPos - armSpeedInc;
//        }
//        if (target > maxArmPos){
//            target = maxArmPos;
//        }
//        if (target < minArmPos){
//            target = minArmPos;
//        }
//        arm.setPower(pidController(target, currentPos));
//
//        return currentPos;
//    }

//    460
//    .37
//    .782
//
//    640
//    .506
//    .782
}
