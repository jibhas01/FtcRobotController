package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArcadeDrive")
public class ArcadeDrive  extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization code goes here
        DcMotor fl = hardwareMap.get(DcMotor.class, "fLeft");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bLeft");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fRight");
        DcMotor br = hardwareMap.get(DcMotor.class, "bRight");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive - forward and back - left stick y-axes
        //turn - turning right and left - right stick x-axes
        //strafe - moving right and left - left stick x-axes
        double drive, turn, strafe;

        double flPower, blPower, frPower, brPower;

        waitForStart();

        while(opModeIsActive()){
            drive = gamepad1.left_stick_y * -1;
            rutn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            frPower = drive - turn - strafe;
            brPower = drive - turn + strafe;

            flPower = drive + turn + strafe;
            blPower = drive + turn - strafe;

            double maxPower = Math.max(Math.abs(frPower),Math.max(Math.abs(brPower),Math.max(Math.abs(flPower),Math.abs(blPower))));
            if(maxPower > 1){
                frPower /= maxPower;
                brPower /= maxPower;
                flPower /= maxPower;
                blPower /= maxPower;
            }

            fr.setPower(frPower);
            br.setPower(brPower);
            fl.setPower(flPower);
            bl.setPower(blPower);

        }

    }
}
