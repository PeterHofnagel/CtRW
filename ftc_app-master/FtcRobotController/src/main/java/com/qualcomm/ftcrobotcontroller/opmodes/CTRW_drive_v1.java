/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 *
 * main Drive Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class CTRW_drive_v1 extends OpMode {

	/*
	 *
	 */
    // Declaring motors and servos

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    Servo rightbaseservo;
    Servo leftbaseservo;
    Servo rightmidservo;
    Servo leftmidservo;
    Servo gripper;
    Servo winch;

    double servoposition = .75;
    double midposition = 0;
    double baseposition = 0;
    double servoSpeed = .01;
    double clipmin = 0;
    double clipmax = 1;
    int calls = 0;




    /**
     * Constructor
     */
    public CTRW_drive_v1() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 *
		 */
        telemetry.addData("hi", " here");
        motorFrontLeft = hardwareMap.dcMotor.get("frontleft"); //port1 left
        motorFrontRight = hardwareMap.dcMotor.get("frontright"); //port1 right
        motorBackLeft = hardwareMap.dcMotor.get("backleft"); //port2 left11a
        motorBackRight = hardwareMap.dcMotor.get("backright"); //port2 right

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);



        rightbaseservo = hardwareMap.servo.get("rightbase"); //port1
        leftbaseservo = hardwareMap.servo.get("leftbase"); //port6
        rightmidservo = hardwareMap.servo.get("rightmid"); //port3
        leftmidservo = hardwareMap.servo.get("leftmid"); //port5
        gripper = hardwareMap.servo.get("gripper"); //port2


        leftmidservo.setDirection(Servo.Direction.REVERSE);
        rightbaseservo.setDirection(Servo.Direction.REVERSE);
        calls=0;
        winch = hardwareMap.servo.get("winch");

        winch.setPosition(0.5);



    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        controlMotors();
        controlArmBase();
        controlArmMid();
        controlGripper();
        controlWinch();
        calls++;
    }

    public void controlMotors()
    {
        double throttle = -gamepad1.left_stick_y;
        double direction = -gamepad1.right_stick_x;
        double right = throttle - direction;
        double left = throttle + direction;
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = scaleInput(right);
        left = scaleInput(left);

        // write the values to the motors
        motorFrontRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
        motorBackRight.setPower(right);
        //telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }

    public void controlArmBase() {


        // if left stick is pressed on gamepad2, move upper arm
        baseposition -= servoSpeed*gamepad2.left_stick_y;
        baseposition = Range.clip(baseposition, 0.0, 1);
        rightbaseservo.setPosition(baseposition);
        leftbaseservo.setPosition(baseposition);
        telemetry.addData("setting armbase servo to ", baseposition);

    }

    public void controlArmMid() {

        // if right joystick is pushed on gamepad2, move forearm up.
        midposition -= servoSpeed*gamepad2.right_stick_y;
        midposition = Range.clip(midposition, clipmin, clipmax);
        rightmidservo.setPosition(midposition);
        leftmidservo.setPosition(midposition);
        telemetry.addData("setting midservos to ", midposition);
    }

    public void controlGripper() {

        if (gamepad2.left_bumper) {
            // if the A button is pushed on gamepad1, increment the position of
            // the arm servo.
            servoposition += servoSpeed;
            servoposition = Range.clip(servoposition, 0, 1);
            gripper.setPosition(servoposition);

            //value when servo_1 is vertical--0.708
            //value when servo_1 is parallel to the ground--0.15


        }telemetry.addData("setting gripper to " + servoposition , calls);




        if (gamepad2.right_bumper) {
            // if the Y button is pushed on gamepad1, decrease the position of
            // the arm servo.
            servoposition -= servoSpeed;
            servoposition = Range.clip(servoposition, 0, 1);
            gripper.setPosition(servoposition);

        }telemetry.addData("setting gripper to " + servoposition, calls);
    }

    public void controlWinch() {
        double throttlewinch = gamepad2.left_trigger - gamepad2.right_trigger;
        double winchpower = (throttlewinch/2) + .5;

        winchpower = Range.clip(winchpower,clipmin,clipmax);
        winch.setPosition(winchpower);
        telemetry.addData("servo power: ",winchpower);
        telemetry.addData("throttle: ",throttlewinch);
    }







	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    public void stop()
    {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
