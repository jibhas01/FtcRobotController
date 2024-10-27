package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Concept: DualPad", group = "Concept")

public class ConceptDualPad extends LinearOpMode {
    
    DualPad gpad;

    @Override
    public void runOpMode() {
        
        gpad = new DualPad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        boolean bToggle = false;
        int bToggleCount = 0;

        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);

            if (gpad.b && !gpad.previous.b) {
                bToggle = !bToggle;
                bToggleCount++;
            }

            telemetry.addData("bToggle", bToggle ? "on" : "off");
            telemetry.addData("bToggleCount", bToggleCount);
            telemetry.addLine("-----");
            telemetry.addData("gpad.b", gpad.b);
            telemetry.addData("gpad.shift.b", gpad.shift.b);
            telemetry.addLine("-----");
            telemetry.addData("gpad", gpad);
            telemetry.addData("gpad.shift", gpad.shift);
            telemetry.update();
        }
    }
}
