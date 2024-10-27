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

//import static org.firstinspires.ftc.teamcode.Minibot.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@TeleOp(name="ATeleop", group="ATeleop")

public class ATeleop extends LinearOpMode {


    public Minibot bot;
    
    @Override
    public void runOpMode() {
        bot = new Minibot(); // replace RobotDrive with Minibot
        bot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Minibot.Pose targetPose = new Minibot.Pose(24, 0);
 
        while (opModeIsActive()) {
            bot.updateTracking();
            
            telemetry.addData("field pose", bot.field);
            telemetry.addData("targetPose", targetPose);
            
            double jx = -gamepad1.left_stick_y - gamepad1.right_stick_y; 
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;

            if (gamepad1.start) {
                if (gamepad1.dpad_up) {
                    bot.setHeading(0);
                }
            }
            
            int targetId = -1;
            
            if(gamepad1.x) {
                targetId = 15; // april tag# on the field
            }
            
            
            // get all the apriltags and iterate
            List<AprilTagDetection> currentDetections =
                    bot.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                
              if (detection.id == targetId) {
                   jw = detection.ftcPose.bearing * 0.02; // 0.02 is based on the trial and error
                    jx = (detection.ftcPose.range - 20) * 0.02; // the robot stops at 20 inches before the april tag
                }
                
                telemetry.addData("tag", bot.format(detection));
            }
            
             if (gamepad1.guide) {
                targetPose = new Minibot.Pose(bot.field);
            }

            
             if (gamepad1.back) {
                bot.driveToPose(targetPose, 0.3);
            }
            else {
                bot.driveXYW(jx, jy, jw);
            }
            
           // bot.driveFieldXYW(jx, jy, jw,0);

            telemetry.addData("Status", "Running");
            telemetry.addData("heading", bot.getHeading());
            telemetry.addData("IMU heading", bot.getIMUHeading());
            telemetry.update();
        }
    }

}
