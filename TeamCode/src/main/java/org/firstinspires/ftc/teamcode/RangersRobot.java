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

    // Motors
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    DcMotorEx arm;
    Servo wrist;
    Servo claw;
    IMU imu;

    // IMU
    static double headingOffset = 0;

    // PID
    double integralSum = 0;
    static double kp = 0.02;
    static double ki = 0;
    static double kd = 0;//0.0003;
    static double f = 0;
    double armTarget = 10;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    final double ticks_in_degrees = 537.7 / 360;

    // Arm
    final double minArmPos = 10;
    final double maxArmPos = 1195;
    double armSpeedInc = 1;
    double maxArmSpeed = 0.5;
    double intakeArmTarget = 10;
    double outtakeArmTarget = 450;
    boolean intakeTargetActivated = false;
    boolean outtakeTargetActivated = false;
    boolean initialOuttakeFlag = true;

    // Wrist
    double currentWristPos = 0;
    double maxWristPos = 0.9;
    double minWristPos = 0;
    double wristSpeedInc = 0.004;
    double intakeWristTarget = 0.025;
    double outtakeWristTarget = 0.5;
    boolean intakeWristTargetActivated = false;
    boolean outtakeWristTargetActivated = false;

    // Claw
    double currentClawPos = 0.85;
    double maxClawPos = 0.85;
    double minClawPos = 0.65;
    double clawSpeedInc = 0.004;
    double intakeClawTarget = 0.735;
    double outtakeClawTarget = 0.85;
    boolean intakeClawTargetActivated = false;
    boolean outtakeClawTargetActivated = false;

    @Override
    public void runOpMode() {

        // Initialization
        lf = initDcMotor(hardwareMap, "fl", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "fr", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "bl", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "br", DcMotor.Direction.FORWARD);
        arm = initDcMotor(hardwareMap, "arm", DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(params);

        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        Gamepad previousGamepad2 = new Gamepad();
        previousGamepad2.copy(gamepad2);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        wrist.setPosition(currentWristPos);
        claw.setPosition(currentClawPos);

        waitForStart();

        while (opModeIsActive()) {

            double currArmPos = arm.getCurrentPosition();

            // Drive the robot
            driveXYW(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Move the arm
            if (gamepad2.left_trigger > 0) {
                armTarget = armTarget + gamepad2.left_trigger + armSpeedInc;
            }
            if(gamepad2.right_trigger > 0) {
                armTarget = armTarget - gamepad2.right_trigger - armSpeedInc;
            }
            if (armTarget > maxArmPos){
                armTarget = maxArmPos;
            }
            if (armTarget < minArmPos){
                armTarget = minArmPos;
            }

            // go to intake position
/*            if(gamepad2.a && !previousGamepad2.a){
                intakeTargetActivated = true;
                currentWristPos = intakeWristTarget;
                currentClawPos = intakeClawTarget;
                previousGamepad2.copy(gamepad2);
            }
            if(intakeTargetActivated){
                double prevDiff = intakeArmTarget - armTarget;
                if(currArmPos < intakeArmTarget)
                    armTarget += armSpeedInc + 1;
                else
                    armTarget = armTarget - armSpeedInc - 1;

                double currDiff = intakeArmTarget - armTarget;
                if(prevDiff * currDiff <= 0 ) {
                    intakeTargetActivated = false;
                    initialOuttakeFlag = true;
                }
            }

            // go to outtake position
            if(gamepad2.b && !previousGamepad2.b){
                outtakeTargetActivated = true;
                //currentWristPos = outtakeWristTarget;
                currentClawPos = outtakeClawTarget;
                previousGamepad2.copy(gamepad2);
            }
            if(outtakeTargetActivated){
                double prevDiff = outtakeArmTarget - armTarget;
                if(currArmPos < outtakeArmTarget)
                    armTarget = armTarget + armSpeedInc + 1;
                else
                    armTarget = armTarget - armSpeedInc - 1;

                double currDiff = outtakeArmTarget - armTarget;
                if(prevDiff * currDiff <= 0) {
                    outtakeTargetActivated = false;
                    currentWristPos = outtakeWristTarget;
                }

                if(initialOuttakeFlag && currArmPos <= 10) {
                    armTarget = 60;
                    claw.setPosition(currentClawPos);
                    wrist.setPosition(currentWristPos - 0.01);
                    sleep(200);
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setTargetPosition(60);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                    while (arm.isBusy()) {
                        telemetry.addData("Arm Position: ", currArmPos);
                        telemetry.update();
                    }
                    arm.setPower(0.2); // hold the arm
                    driveXYW(0.7, 0, 0);
                    sleep(700);
                    driveXYW(0, 0, 0);
                    initialOuttakeFlag = false;
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currArmPos = arm.getCurrentPosition();
                }

            }

            if(!gamepad2.a) previousGamepad2.copy(gamepad2);
            if(!gamepad2.b) previousGamepad2.copy(gamepad2);
*/
            arm.setPower(pidController(armTarget, currArmPos));

            // Move the wrist
//            double relativeWristPosForwards = 7.88 * 0.000001 * Math.pow(arm.getCurrentPosition(), 2)
//                    + 0.00132 * arm.getCurrentPosition() + 0.2528;
//            if(arm.getCurrentPosition() < 175 && (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0))
//                currentWristPos = relativeWristPosForwards;
//
//            double relativeWristPosBackwards = 0.00264 * arm.getCurrentPosition() - 2.500;
//
//            if(arm.getCurrentPosition() > 1025 &&
//                    (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) &&
//                    currentWristPos < relativeWristPosBackwards)// && currentWristPos < 0.621)
//                currentWristPos = relativeWristPosBackwards;//0.621;

            if(gamepad2.left_stick_y > 0 && currentWristPos < maxWristPos){
                currentWristPos += wristSpeedInc;
//                if(arm.getCurrentPosition() < 175 && currentWristPos > relativeWristPosForwards){
//                    currentWristPos = relativeWristPosForwards;
//                }
            }else if(gamepad2.left_stick_y < 0 && currentWristPos > minWristPos){
                currentWristPos -= wristSpeedInc;
//                if(arm.getCurrentPosition() > 1025 && currentWristPos < relativeWristPosBackwards - 0.03){
//                    currentWristPos = relativeWristPosBackwards - 0.03;
//                }
            }

            telemetry.addData("currentWristPos ", currentWristPos);
            wrist.setPosition(currentWristPos);

            // Move the claw --- max open 0.65, max close 0.78
            if(gamepad2.right_stick_x > 0 && currentClawPos < maxClawPos){
                currentClawPos += clawSpeedInc;
            }else if(gamepad2.right_stick_x < 0 && currentClawPos > minClawPos){
                currentClawPos -= clawSpeedInc;
            }
            claw.setPosition(currentClawPos);

            //** Reset the heading
            if (gamepad1.y && previousGamepad1.y) {
                setHeading(0);
                telemetry.addData("Status" , "Reset Heading");
                previousGamepad1.copy(gamepad1);
            }
            previousGamepad1 = gamepad1;

            telemetry.addData("Status", "Running");
            telemetry.addData("IMU heading", getIMUHeading());
            telemetry.addData("wrist pos: ", wrist.getPosition());
            telemetry.addData("claw pos: ", claw.getPosition());
            telemetry.update();
        }
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

        double maxPower = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower),
                        Math.max(Math.abs(lbPower), Math.abs(rbPower))));

        if(maxPower > 1){
            lfPower /= maxPower;
            rfPower /= maxPower;
            lbPower /= maxPower;
            rbPower /= maxPower;
        }

        lf.setPower(lfPower * 0.40);
        rf.setPower(rfPower * 0.40);
        lb.setPower(lbPower * 0.40);
        rb.setPower(rbPower * 0.40);
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

        double absPid = Math.abs(pid);
        double pidSign = pid / absPid;

        if(absPid > maxArmSpeed) pid = maxArmSpeed * pidSign;

        return pid + ff;
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
