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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Drive", group="Iterative Opmode")

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime dt = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();
    private PID armPid = new PID(8.0, 0.00001, 500.0);
    private double speed = 0.5;

    private double drive = 0.0;
    private double strafe = 0.0;
    private double rotate = 0.0;
    private double armPower = 0.0;
    private int armTarget = 0;

    private boolean arm_usePid = false;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Set sucker servos to neutral position.
        robot.rightClaw.setPosition(0.5);
        robot.leftClaw.setPosition(0.5);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        while (robot.rightArm.getCurrentPosition() > robot.LEFT_LEV3) {
            robot.rightArm.setPower(-0.2);
            robot.leftArm.setPower(0.2);
        }
        robot.rightArm.setPower(0.0);
        robot.leftArm.setPower(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Choose whether to drive with joystick or nudge with dpad
        if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right) {
            drive = (gamepad1.dpad_down ? 0.2 : 0) - (gamepad1.dpad_up ? 0.2 : 0);
            strafe = 0.0;
            rotate = (gamepad1.dpad_right ? 0.4 : 0) - (gamepad1.dpad_left ? 0.4 : 0);
        } else {
            drive = Math.pow(gamepad1.left_stick_y, 3) * speed;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x * speed;
        }
        robot.mecanumDrive(drive, strafe, rotate);

        // Check this first or else we won't be able to spit the block out.
        if (gamepad1.left_bumper) {
            robot.rightClaw.setPosition(0.0);
            robot.leftClaw.setPosition(1.0);
        // Turn off if the limit switch is hit
        } else if (!robot.grabberLimit.getState()) {
            robot.rightClaw.setPosition(0.5);
            robot.leftClaw.setPosition(0.5);
        // Suck the lego piece in.
        } else if (gamepad1.right_bumper) {
            robot.rightClaw.setPosition(1.0);
            robot.leftClaw.setPosition(0.0);
        // Slow inward creep by default
        } else {
            robot.rightClaw.setPosition(0.6);
            robot.leftClaw.setPosition(0.4);
        }

//        if (gamepad1.right_trigger > .1 || gamepad1.left_trigger > .1)
//            armPower = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);
//        else
//            armPower = 0.0;
//        robot.leftArm.setPower(armPower);
//        robot.rightArm.setPower(-armPower);

        if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
            arm_usePid = false;

            armPower = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);

            robot.leftArm.setPower(armPower);
            robot.rightArm.setPower(-armPower);

        } else if (gamepad1.a) {
            arm_usePid = true;
            armTarget = robot.RIGHT_GRAB;
        } else if (gamepad1.x) {
            arm_usePid = true;
            armTarget = robot.RIGHT_LEV1;
        } else if (gamepad1.y) {
            arm_usePid = true;
            armTarget = robot.RIGHT_BRIDGE;
        } else if (gamepad1.b) {
            arm_usePid = true;
            armTarget = robot.RIGHT_LEV3;
        } else if (!arm_usePid) { // Hold position after releasing triggers
            arm_usePid = true;
            armTarget = robot.rightArm.getCurrentPosition();
        }

        if (arm_usePid) {
            armPid.setError(armTarget - robot.rightArm.getCurrentPosition());
            robot.rightArm.setPower(armPid.power());
            robot.leftArm.setPower(-armPid.power());
        }


//        telemetry.addData("Left claw pos:", robot.leftClaw.getPosition());
//        telemetry.addData("Right claw pos:", robot.rightClaw.getPosition());
        telemetry.addData("Claw pos:", robot.rightClaw.getPosition());
        telemetry.addData("Grabber limit switch:",
                robot.grabberLimit.getState() ? "not pressed" : "pressed");
        telemetry.addLine();
        telemetry.addData("Right arm pos:", robot.rightArm.getCurrentPosition());
        telemetry.addData("Right arm power", robot.rightArm.getPower());
//        telemetry.addData("right arm mode", robot.leftArm.getMode());
//        telemetry.addData("left arm mode", robot.leftArm.getMode());
        telemetry.addData("PID target", armTarget);
        telemetry.addData("PID error", armTarget - robot.rightArm.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("PID Proportional", armPid.getP());
        telemetry.addData("PID Integral", armPid.getI());
        telemetry.addData("PID Derivative", armPid.getD());
        telemetry.addLine();
        telemetry.addData("right front wheel speed",
                (robot.rightFrontDrive.getCurrentPosition() - robot.rightFrontLastPos) / dt.seconds());
        telemetry.addData("right rear wheel speed",
                (robot.rightRearDrive.getCurrentPosition() - robot.rightRearLastPos) / dt.seconds());
        telemetry.addData("left front wheel speed",
                (robot.leftFrontDrive.getCurrentPosition() - robot.leftFrontLastPos) / dt.seconds());
        telemetry.addData("left rear wheel speed",
                (robot.leftRearDrive.getCurrentPosition() - robot.leftRearLastPos) / dt.seconds());

        robot.rightFrontLastPos = robot.rightFrontDrive.getCurrentPosition();
        robot.rightRearLastPos = robot.rightRearDrive.getCurrentPosition();
        robot.leftFrontLastPos = robot.leftFrontDrive.getCurrentPosition();
        robot.leftRearLastPos = robot.leftRearDrive.getCurrentPosition();
        dt.reset();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
