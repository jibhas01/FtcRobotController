package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDrive")
public class TankDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization code goes here
        DcMotor fl = hardwareMap.get(DcMotor.class, "fLeft");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bLeft");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fRight");
        DcMotor br = hardwareMap.get(DcMotor.class, "bRight");
        
        //motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //drive - forward and back - left stick y-axes
        //turn - right and left - right stick x-axes
        double drive, turn;
        double rightsidePower, leftsidePower;

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;

            rightsidePower = drive - turn;
            leftsidePower = drive + turn;

            double max = Math.max(Math.abs(rightsidePower), Math.abs(leftsidePower));

            fl.setPower(leftsidePower/max);
            bl.setPower(leftsidePower/max);
            fr.setPower(rightsidePower);
            br.setPower(rightsidePower);

        }
    }
}
