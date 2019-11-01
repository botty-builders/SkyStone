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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="BasicDrive", group="Pushbot")
//@Disabled
public class BasicDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private BasicHardwareMap robot           = new BasicHardwareMap();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    private ElapsedTime runtime = new ElapsedTime();
    static double windUp = 0;
    private AndroidAccelerometer accel = new AndroidAccelerometer();

    double currentMotorPower;



    @Override
    public void runOpMode() {
//        double left_x;
        double left_y;
        double right_x;
//        double right_y;
        double drive;
        double turn;
        double max;
        double left;
        double right;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Botty Builders");    //
        if (accel.isAvailable()) {
            accel.startListening();
            telemetry.addData("Say", "Good Accelerometer");
        } else {
            telemetry.addData("Say", "No Accelerometer Detected");
        }
        telemetry.update();

        //robot.leftDrive.setPower(0);
        //currentMotorPower = robot.leftDrive.getPower();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            //left_x = -gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            right_x  =  gamepad1.right_stick_x;

            drive = left_y;
            turn = right_x;

            windUp = windUp + gamepad1.left_stick_y;
            if (gamepad1.a) {
                windUp = 0;
            }


            // Combine drive and turn for blended motion.
            left  = - drive - turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Use gamepad y and x to control arm up and down, respectively
            if (gamepad1.x) robot.arm.setPower(-0.3);
            else if (gamepad1.y) robot.arm.setPower(0.4);
            else robot.arm.setPower(0.0);

            // Use right bumper to start drum rotation
            if (gamepad1.right_bumper) robot.drum.setPower(-1.0);
            else robot.drum.setPower(0.0);

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            // Use b and a to move the flipper up and down, respectively
            if (gamepad1.a) robot.flipper.setPosition(BasicHardwareMap.FLIPPER_DOWN);
            else if (gamepad1.b) robot.flipper.setPosition(BasicHardwareMap.FLIPPER_UP);

            // Use gamepad left & right Bumpers to open and close the claw
            /*
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.y)
                robot.leftArm.setPower(robot.ARM_UP_POWER);
            else if (gamepad1.a)
                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
            else
                robot.leftArm.setPower(0.0);
*/

            double left_stick_x = gamepad1.left_stick_x + gamepad2.left_stick_x;
            double left_stick_y = gamepad1.left_stick_y + gamepad2.left_stick_y;
            double right_stick_x = gamepad1.right_stick_x + gamepad2.right_stick_x;
            double right_stick_y = gamepad1.right_stick_y + gamepad2.right_stick_y;

            // Send telemetry message to signify robot running;
           // telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Game Pad"," ");
            telemetry.addData("left x",  "%.2f", left_stick_x);
            telemetry.addData("left y",  "%.2f", left_stick_y);
            telemetry.addData("right x",  "%.2f", right_stick_x);
            telemetry.addData("right y",  "%.2f", right_stick_y);
            telemetry.addData("Left Motor", "%.2f", robot.leftDrive.getPower());
            telemetry.addData("Right Motor", "%.2f", robot.rightDrive.getPower());
            telemetry.addData("Button A", gamepad1.a);
            telemetry.addData("Button B", gamepad1.b);
            telemetry.addData("Button X", gamepad1.x);
            telemetry.addData("Button Y", gamepad1.y);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("WindUp: ",windUp);

            telemetry.addData(" "," ");
            telemetry.addData("Accelerometer Data (X): ",  "%.5f", accel.getX());
            telemetry.addData("Accelerometer Data (Y): ",  "%.5f", accel.getY());
            telemetry.addData("Accelerometer Data (Z): ",  "%.5f", accel.getZ());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
