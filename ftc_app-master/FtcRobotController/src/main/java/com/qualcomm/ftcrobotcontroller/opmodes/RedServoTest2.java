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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class RedServoTest2 extends OpMode {

	/*
	 * Tests motor function
	 * Pressing a,b,x,y on the gamepad moves the motors individually
	 */



	Servo right;
	Servo left;
	double rightposition = 0;
	double leftposition = 0;
	int calls = 0;



	/**
	 * Constructor
	 */
	public RedServoTest2() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        telemetry.addData("hi", " there");
		right = hardwareMap.servo.get("servo_1");
		left = hardwareMap.servo.get("servo_6");
		left.setDirection(Servo.Direction.REVERSE);
		calls=0;

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
    }
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

		calls++;

		// update the position of the arm.
		if (gamepad1.a) {
			// if the A button is pushed on gamepad1, increment the position of
			// the arm servo.
			rightposition += 0.001;
			rightposition = Range.clip(rightposition, 0.1, 0.99);
			right.setPosition(rightposition);
			telemetry.addData("setting servo_1, right servo to " + rightposition , calls);
			//value when servo_1 is vertical--0.708
			//value when servo_1 is parallel to the ground--0.15
			leftposition += 0.001;
			leftposition = Range.clip(leftposition, 0.1, 0.99);
			left.setPosition(leftposition);
			telemetry.addData("setting servo_6, left servo to " + leftposition, calls);

		}

		if (gamepad1.y) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the arm servo.
            leftposition -= 0.001;
            leftposition = Range.clip(leftposition, 0.1, 0.99);
            left.setPosition(leftposition);
            telemetry.addData("setting servo_6, left servo to " + leftposition, calls);
			//leftposition += 0.001;
			//leftposition = Range.clip(leftposition, 0.1, 0.99);
			//left.setPosition(leftposition);
			//telemetry.addData("setting servo_6, left servo to " + leftposition, calls);
			//value when servo_6 is completely vertical--0.737
			//value when servo_6 is parallel to the ground--0.137
			rightposition -= 0.001;
			rightposition = Range.clip(rightposition, 0.1, 0.99);
			right.setPosition(rightposition);
			telemetry.addData("setting servo_1, right servo to " + rightposition, calls);
		}
		//difference between servos when vertical= -0.029
		//difference between servos when parallel to the ground= 0.013
		if (gamepad1.x) {
			// if the A button is pushed on gamepad1, increment the position of
			// the arm servo.
			//rightposition -= 0.001;
			//rightposition = Range.clip(rightposition, 0.1, 0.99);
			//right.setPosition(rightposition);
			//telemetry.addData("setting servo_1, right servo to " + rightposition, calls);
		}

		if (gamepad1.b) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the arm servo.
			//leftposition -= 0.001;
            //leftposition = Range.clip(leftposition, 0.1, 0.99);
            //left.setPosition(leftposition);
            //telemetry.addData("setting servo_6, left servo to " + leftposition, calls);

		}
/*
servo_1 90deg = 0.58
 */
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}
}
