package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "First_Autonomous")
public class FirstAuto extends LinearOpMode {

    DcMotor fl = null;
    DcMotor bl = null;
    DcMotor fr = null;
    DcMotor br = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization here
        fl = hardwareMap.get(DcMotor.class, "fLeft");
        bl = hardwareMap.get(DcMotor.class, "bLeft");
        fr = hardwareMap.get(DcMotor.class, "fRight");
        br = hardwareMap.get(DcMotor.class, "bRight");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // drive to fist target -- adjust the numbers
        move(1000, 1000, 0.5);

        // turn - 90 degrees left -- adjust the numbers
        move(-500, 500, 0.5);

        // drive to second target -- adjust the numbers
        move(1000, 1000, 0.5);

        // turn - 45 degrees left -- adjust the numbers
        move(0, 250, 0.5);

        // drive to hit the ball -- adjust the numbers
        move(500, 500, 0.5);

        // reverse everything to go back home -- adjust the numbers
        move(-500, -500, 0.5);
        move(250, -250, 0.5);
        move(-1000, -1000, 0.5);
        move(500, -500, 0.5);
        move(-1000, -1000, 0.5);

    }

    public void move(int lTarget, int rTarget, double speed){
        fl.setTargetPosition(lTarget);
        bl.setTargetPosition(lTarget);

        fr.setTargetPosition(rTarget);
        br.setTargetPosition(rTarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(speed);
        bl.setPower(speed);
        fr.setPower(speed);
        br.setPower(speed);

        while (opModeIsActive() && (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            idle();
        }
    }

}
