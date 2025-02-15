
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RRAutoRight", group="Robot")
public class RRAutoRight extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;
    private DcMotorEx bl;
    private DcMotorEx br;
    DcMotorEx arm;
    Servo wrist;
    Servo claw;

    // PID
    double integralSum = 0;
    static double kp = 0.03;
    static double ki = 0;
    static double kd = 0;
    static double f = 0;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    final double ticks_in_degrees = 537.7 / 360;

    private ElapsedTime     runtime = new ElapsedTime();

    double currentWristPos = 0;
    double currentClawPos = 0.85;
    //double armTarget = 0;
    double maxArmSpeed = 0.5;
    double armSpeedInc = 2;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fl = initDcMotor(hardwareMap, "fl", DcMotor.Direction.FORWARD);
        fr = initDcMotor(hardwareMap, "fr", DcMotor.Direction.REVERSE);
        bl = initDcMotor(hardwareMap, "bl", DcMotor.Direction.FORWARD);
        br = initDcMotor(hardwareMap, "br", DcMotor.Direction.REVERSE);
        arm = initDcMotor(hardwareMap, "arm", DcMotor.Direction.FORWARD);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");

        wrist.setPosition(currentWristPos);
        claw.setPosition(currentClawPos);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                fl.getCurrentPosition(),
                fr.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        moveArm2(200, 0, 0.85);
        //driveWithEncoders(700, 0, 0, 5);
        moveArm2(-150, 0, 0.85);
        //driveWithEncoders(-250, 0, 0, 5);
        //driveWithEncoders(0, -300, 0, 5);

        //telemetry.addData("Path", "Complete");
        //telemetry.update();
        sleep(30000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    private void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = fl.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = fr.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fl.setTargetPosition(newLeftTarget);
            fr.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fl.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fl.isBusy() && fr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        fl.getCurrentPosition(), fr.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fl.setPower(0);
            fr.setPower(0);

            // Turn off RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    private DcMotorEx initDcMotor(HardwareMap hardwareMap,
                                  String name,
                                  DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return m;
    }

    private void driveWithEncoders(int rx, int ry, int rw, double timeoutS){
        int newFlTarget = fl.getCurrentPosition() + rx - ry - rw;
        int newFrTarget = fr.getCurrentPosition() + rx + ry + rw;
        int newBlTarget = bl.getCurrentPosition() + rx + ry - rw;
        int newBrTarget = br.getCurrentPosition() + rx - ry + rw;

        fl.setTargetPosition(newFlTarget);
        fr.setTargetPosition(newFrTarget);
        bl.setTargetPosition(newBlTarget);
        br.setTargetPosition(newBrTarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        fl.setPower(0.3);
        fr.setPower(0.3);
        bl.setPower(0.3);
        br.setPower(0.3);

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                    newFlTarget,  newFrTarget, newBlTarget, newBrTarget);
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    fl.getCurrentPosition(), fr.getCurrentPosition(),
                    bl.getCurrentPosition(), br.getCurrentPosition());
            telemetry.update();
        }

        sleep(250);
    }

    private void moveArm(int armPosition, double wristPosition, double clawPosition){

        int newArmTarget = arm.getCurrentPosition() + armPosition;
        arm.setTargetPosition(newArmTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        arm.setPower(0.3);

//        if(armPosition == 10)
//            arm.setPower(pidController(armPosition, arm.getCurrentPosition()));
        wrist.setPosition(wristPosition);
        claw.setPosition(clawPosition);
        sleep(5000);

        // Display it for the driver.
        telemetry.addData("i ",  " %7d :%7d", armPosition,  arm.getCurrentPosition());
        telemetry.update();


    }

    private void moveArm2(double armTarget, double wristPosition, double clawPosition){

//        int newArmTarget = arm.getCurrentPosition() + armTarget;
//        arm.setTargetPosition(newArmTarget);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        runtime.reset();
//
//        arm.setPower(0.3);
//
////        if(armTarget == 10)
////            arm.setPower(pidController(armTarget, arm.getCurrentPosition()));
//        wrist.setPosition(wristPosition);
//        claw.setPosition(clawPosition);
//        sleep(5000);



        double interimTarget = 0;

        while(opModeIsActive()){
            double curArmPos = arm.getCurrentPosition();
            if(armTarget > curArmPos)
                interimTarget = interimTarget + armSpeedInc;
            else if(armTarget < curArmPos)
                interimTarget = interimTarget - armSpeedInc;
            else
                interimTarget = interimTarget + 0;

            arm.setPower(pidController(interimTarget, curArmPos));

            // Display it for the driver.
            telemetry.addData("arm at ",  " %7f :%7f", armTarget,  curArmPos);
            telemetry.update();
        }


    }

    private double pidController(double target, double state){
        double error = target - state;
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;
        elapsedTime.reset();
        double pid = (error * kp) + (derivative * kd) + (integralSum * ki);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        telemetry.addData("pid: ", pid);
        telemetry.addData("ff: ", ff);
        telemetry.addData("target: ", target);
        telemetry.addData("armPos", state);

        double absPid = Math.abs(pid);
        double pidSign = pid / absPid;

        if(absPid > maxArmSpeed) pid = maxArmSpeed * pidSign;

        return pid + ff;
    }

}
