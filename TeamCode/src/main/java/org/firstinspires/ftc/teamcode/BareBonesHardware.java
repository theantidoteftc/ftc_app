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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Based on org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot

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
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class BareBonesHardware
{
    /* Public OpMode members. */
    public DcMotor  hexFrontLeft   = null;
    public DcMotor  hexFrontRight  = null;
    public DcMotor  hexRearLeft  = null;
    public DcMotor  hexRearRight  = null;
    public DcMotor  hexSlide  = null;
    public DcMotor  pivotMotor  = null;

    public Servo leftHook = null; //3 on the left
    public Servo rightHook = null; //4 on the right
    public Servo leftBlock = null; //1 on the left
    public Servo rightBlock = null; //1 on the right
    public Servo mineralBlock = null; //2 on the left
    public Servo intakeServo = null; //0 on the right
    public Servo b = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public BareBonesHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        hexFrontLeft  = hwMap.get(DcMotor.class, "hexFrontLeft");
        hexFrontRight = hwMap.get(DcMotor.class, "hexFrontRight");
        hexRearLeft  = hwMap.get(DcMotor.class, "hexRearLeft");
        hexRearRight = hwMap.get(DcMotor.class, "hexRearRight");
        hexSlide  = hwMap.get(DcMotor.class, "hexSlide");
        pivotMotor = hwMap.get(DcMotor.class, "pivotMotor");

        // Define and Initialize Servos

        leftHook = hwMap.get(Servo.class, "leftHook");
        rightHook = hwMap.get(Servo.class, "rightHook");
        leftBlock = hwMap.get(Servo.class, "leftBlock");
        rightBlock = hwMap.get(Servo.class, "rightBlock");
        mineralBlock = hwMap.get(Servo.class, "mineralBlock");
        intakeServo = hwMap.get(Servo.class, "intakeServo");

        hexFrontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        hexFrontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        hexRearLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        hexRearRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        hexSlide.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        hexFrontLeft.setPower(0);
        hexFrontRight.setPower(0);
        hexRearLeft.setPower(0);
        hexRearRight.setPower(0);
        hexSlide.setPower(0);
        pivotMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hexFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hexFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hexRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hexRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hexSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
 }

