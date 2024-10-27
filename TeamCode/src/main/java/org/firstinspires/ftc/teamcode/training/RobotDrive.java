package org.firstinspires.ftc.teamcode.training;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.training.RobotValues.*;


public class RobotDrive {

    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    IMU imu;
    static double headingOffset = 0;

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
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);
    }
    
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    public void setHeading(double h) {
        headingOffset = h - getIMUHeading();
    }

    public double getHeading() {
        return AngleUnit.normalizeDegrees(headingOffset + getIMUHeading());
    }
    
    public void init(HardwareMap hardwareMap) {
        lf = initDcMotor(hardwareMap, "lf", LEFTDIR);
        rf = initDcMotor(hardwareMap, "rf", RIGHTDIR);
        lb = initDcMotor(hardwareMap, "lb", LEFTDIR);
        rb = initDcMotor(hardwareMap, "rb", RIGHTDIR);
        initIMU(hardwareMap);
    }
    
    
     
     public void driveFieldXYW(double fx, double fy, double fw) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading());
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
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

    public void setEncoders(boolean toggle){
        if(!toggle){
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}