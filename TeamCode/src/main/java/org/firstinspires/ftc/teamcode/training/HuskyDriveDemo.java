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
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: HuskyDriveDemo", group="HuskyDriveDemo")

public class HuskyDriveDemo extends LinearOpMode {


    public RobotDrive bot;
    public HuskyLens huskylens;
    
    @Override
    public void runOpMode() {
        bot = new RobotDrive();
        bot.init(hardwareMap);

        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double jx = -gamepad1.left_stick_y - gamepad1.right_stick_y; 
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;
            
            int targetId = -1;
            if (gamepad1.x) targetId = 1;
            if (gamepad1.y) targetId = 2;
            if (gamepad1.a) targetId = 3;
            
            HuskyLens.Block[] currentBlocks = huskylens.blocks();
            telemetry.addData("blocks.length", currentBlocks.length);

            for(HuskyLens.Block block : currentBlocks) {
                int center = block.left + block.width / 2;
                if (block.id == targetId) {
                    jw = (160 - center) * 0.004;
                    
                    if (gamepad1.right_bumper) {
                        jx = (220 - block.top) * 0.002;
                    }
                }
                telemetry.addData("block", block.toString());
            }

            bot.driveXYW(jx, jy, jw);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

}
