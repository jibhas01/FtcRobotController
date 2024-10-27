package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Jib's PIDTutorial", group="Linear OpMode")

public class PIDTutorial extends LinearOpMode{

    IMU imu;
    public RobotDrive bot;

    double integralSum = 0;
    double kp = 0.5;
    double ki = 0.5;
    double kd = 0.5;
    double lastError = 0;

    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new RobotDrive();
        bot.init(hardwareMap);
        bot.setEncoders(false);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double targetAngle = Math.toRadians(90);
        while(opModeIsActive()){
            double power = pidController(targetAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            bot.driveXYW(0, 0, -power);
        }


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
