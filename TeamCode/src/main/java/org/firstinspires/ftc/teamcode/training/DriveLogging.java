/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriveLogging", group="DriveLogging")

public class DriveLogging extends LinearOpMode {


    public RobotDrive bot;
    public SimpleLogger log;
      
    @Override
    public void runOpMode() {
        bot = new RobotDrive();
        bot.init(hardwareMap);
        
        log = new SimpleLogger();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        log.open();
        log.add("loop").add("apress")
            .add("lf")
            .headerLine();
        int loopCount = 0;
        int logUntil = 0;

        while (opModeIsActive()) {
            loopCount++;
            double jx = -gamepad1.left_stick_y - gamepad1.right_stick_y; 
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;

            boolean apress = gamepad1.a;
            if (apress) {
               jx = -0.5;
               logUntil = loopCount + 100;
            }
            
            if (loopCount < logUntil) {
               log.add(loopCount).add(apress)
                  .add(bot.lf.getCurrentPosition())
                  .tsLine();
            }
            
            bot.driveXYW(jx, jy, jw);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

}
