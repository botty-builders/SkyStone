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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private double speed = 0.3;

    private double drive = 0.0;
    private double strafe = 0.0;
    private double rotate = 0.0;
    private double armPower = 0.0;

    private int rightArmTemp = robot.RIGHT_LEV3;
    private int leftArmTemp = robot.LEFT_LEV3;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Choose whether to drive with joystick or nudge with dpad
        // TODO: uncomment and edit dpad nudge code after arm positions are set.
        // TODO: increase nudge drive speed.
//        if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right) {
//            drive = (gamepad1.dpad_down ? 0.1 : 0) - (gamepad1.dpad_up ? 0.1 : 0);
//            strafe = (gamepad1.dpad_right ? 0.4 : 0) - (gamepad1.dpad_left ? 0.4 : 0);
//            rotate = 0.0;
//        } else {
            drive = Math.pow(gamepad1.left_stick_y, 3) * speed;
            strafe = gamepad1.left_stick_x;
            rotate = Math.pow(gamepad1.right_stick_x, 3) * speed;
//        }
        robot.mecanumDrive(drive, strafe, rotate);

        // TODO: use right bumper to close the claw, left bumper to open the claw.
        // Check this first or else we won't be able to spit the block out.
        if (gamepad1.left_bumper) {
            robot.rightClaw.setPosition(1.0);
//            robot.leftClaw.setPosition(0.0);

        } else if (gamepad1.right_bumper) {
            robot.rightClaw.setPosition(0.0);
//            robot.leftClaw.setPosition(1.0);

        }

        if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) { // set power for manual control
            robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armPower = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.75;

            robot.leftArm.setPower(armPower);
            robot.rightArm.setPower(-armPower);
        } else if (robot.rightArm.isBusy() || robot.leftArm.isBusy()) {
            // Set power for run to position
            robot.leftArm.setPower(1.0);
            robot.rightArm.setPower(-1.0);
        } else {
            // Default to zero power
            robot.leftArm.setPower(0.0);
            robot.rightArm.setPower(0.0);
        }

        if (gamepad1.a) {
            robot.rightArm.setTargetPosition(robot.RIGHT_GRAB);
            robot.leftArm.setTargetPosition(robot.LEFT_GRAB);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.x) {
            robot.rightArm.setTargetPosition(robot.RIGHT_LEV1);
            robot.leftArm.setTargetPosition(robot.LEFT_LEV1);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.y) {
            robot.rightArm.setTargetPosition(robot.RIGHT_BRIDGE);
            robot.leftArm.setTargetPosition(robot.LEFT_BRIDGE);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.b) {
            robot.rightArm.setTargetPosition(robot.RIGHT_LEV3);
            robot.leftArm.setTargetPosition(robot.LEFT_LEV3);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // TODO: remove extra dpad code when arm positions are set.
        if (gamepad1.dpad_up) {
            rightArmTemp++;
            leftArmTemp--;
            robot.rightArm.setTargetPosition(rightArmTemp);
            robot.leftArm.setTargetPosition(leftArmTemp);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.dpad_down) {
            rightArmTemp--;
            leftArmTemp++;
            robot.rightArm.setTargetPosition(rightArmTemp);
            robot.leftArm.setTargetPosition(leftArmTemp);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // TODO: find which telemetry updates are unnecessary and remove them to avoid clutter.
        telemetry.addData("Claw pos:", robot.rightClaw.getPosition());
        telemetry.addData("Grabber limit switch:",
                robot.grabberLimit.getState() ? "not pressed" : "pressed");
        telemetry.addLine();
        telemetry.addData("Right arm pos:", robot.rightArm.getCurrentPosition());
        telemetry.addData("Left arm pos:", robot.leftArm.getCurrentPosition());
        telemetry.addData("Right arm target pos:", robot.rightArm.getTargetPosition());
        telemetry.addData("Left arm target pos:", robot.leftArm.getTargetPosition());
        telemetry.addLine();
        telemetry.addData("Right arm power", robot.rightArm.getPower());
        telemetry.addLine();
        telemetry.addData("right front wheel speed", "%.6f",
                (robot.rightFrontDrive.getCurrentPosition() - 0 * robot.rightFrontLastPos));
        telemetry.addData("right rear wheel speed", "%.6f",
                (robot.rightRearDrive.getCurrentPosition() - 0 * robot.rightRearLastPos));
        telemetry.addData("left front wheel speed", "%.6f",
                (robot.leftFrontDrive.getCurrentPosition() - 0 * robot.leftFrontLastPos));
        telemetry.addData("left rear wheel speed", "%.6f",
                (robot.leftRearDrive.getCurrentPosition() - 0 * robot.leftRearLastPos));
        telemetry.addData("dt", "%.6f", dt.seconds());


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
