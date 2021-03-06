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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  rightFrontDrive   = null;
    public DcMotor  rightRearDrive   = null;
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotorEx  leftArm   = null;
    public DcMotorEx  rightArm   = null;

    // TODO: find which servo is unused and delete it from the code.
    public Servo rightClaw = null;

    public DigitalChannel grabberLimit;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final double RIGHT_SERVO_STOWED    =  1.0 ;
    public static final double RIGHT_SERVO_CLOSED    =  0.2 ;
    public static final double RIGHT_SERVO_OPEN      =  0.45 ;
    public static final double LEFT_SERVO_STOWED     =  0.0 ;
    public static final double LEFT_SERVO_CLOSED     =  0.8 ;
    public static final double LEFT_SERVO_OPEN       =  0.55 ;

    // TODO: set discrete positions of redesigned arm.
    public static final int LEFT_POS1 = 132;
    public static final int LEFT_POS2 = 116;
    public static final int LEFT_POS3 = 101;
    public static final int LEFT_POS4 = 93;
    public static final int RIGHT_POS1 = -157;
    public static final int RIGHT_POS2 = -141;
    public static final int RIGHT_POS3 = -126;
    public static final int RIGHT_POS4 = -118;

    public double rightFrontLastPos = 0.0;
    public double leftFrontLastPos = 0.0;
    public double rightRearLastPos = 0.0;
    public double leftRearLastPos = 0.0;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        rightFrontDrive = hwMap.get(DcMotor.class, "front_right");
        leftFrontDrive = hwMap.get(DcMotor.class, "front_left");
        leftRearDrive = hwMap.get(DcMotor.class, "bottom_left");
        rightRearDrive = hwMap.get(DcMotor.class, "bottom_right");
        rightArm = (DcMotorEx) hwMap.get(DcMotorEx.class, "rightArm");
        leftArm = (DcMotorEx) hwMap.get(DcMotorEx.class, "leftArm");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftArm.setPower(0);
        rightArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Custom PID coefficients for the arm
        // BE WARNED: DO NOT TOUCH THE I, D, OR F TERMS!  THERE ARE CONSEQUENCES!!!
        leftArm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(50.0, 0.0, 0.0, 0.0));
        rightArm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(50.0, 0.0, 0.0, 0.0));

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        rightClaw = hwMap.get(Servo.class, "rightClaw");

        // Define and initialize all installed sensors
        grabberLimit = hwMap.get(DigitalChannel.class, "touchSensor");
        grabberLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    public void mecanumDrive(double drive, double strafe, double rotate) {
        double FR = drive + strafe + rotate;
        double FL = drive - strafe - rotate;
        double RL = drive + strafe - rotate;
        double RR = drive - strafe + rotate;

        // Normalize drive variables so one of the motors doesn't peak too soon.
        double max = Math.max(
                Math.max(Math.abs(FR), Math.abs(FL)),
                Math.max(Math.abs(RL), Math.abs(RR)));
        if (max > 1) {
            FR /= max;
            FL /= max;
            RL /= max;
            RR /= max;
        }

        rightFrontDrive.setPower(FR);
        leftFrontDrive.setPower(FL);
        leftRearDrive.setPower(RL);
        rightRearDrive.setPower(RR);
    }
}

