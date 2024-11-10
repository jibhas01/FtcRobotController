package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PIDDemo01", group="Linear OpMode")
public class PIDDemo extends LinearOpMode {

    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;

    IMU imu;

    double integralSum = 0;
    double kp = 0.5;
    double ki = 0.5;
    double kd = 0.5;
    double lastError = 0;

    double targetAngle = Math.toRadians(90);

    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize
        lf = initDcMotor(hardwareMap, "lf", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "rf", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "lb", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "rb", DcMotor.Direction.FORWARD);
        initIMU(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double power = pidController(targetAngle, currentAngle);

            telemetry.addData("current angle: ", currentAngle);
            telemetry.addData("target angle: ", targetAngle);
            telemetry.addData("rw power: ", power);
            telemetry.update();

            driveXYW(0, 0, -power);
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

    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(params);
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

    public double pidController(double target, double state){
        double error = angleWrap(target - state);
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;

        elapsedTime.reset();

        return (error * kp) + (derivative * kd) + (integralSum * ki);
    }

    public double angleWrap(double radians){
        while (radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }

        return radians;
    }
}
