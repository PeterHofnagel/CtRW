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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class RedDriveFinalSensor extends OpMode {

	/*
	 *
	 */
	// TETRIX VALUES.
//Test Comment
	DcMotor motorFrontRight;
	DcMotor motorFrontLeft;
	DcMotor motorBackRight;
	DcMotor motorBackLeft;
	Servo rightbaseservo;
	Servo leftbaseservo;
    Servo rightmidservo;
    Servo leftmidservo;
    Servo gripper;
TouchSensor touch;
	OpticalDistanceSensor eyes;
	ColorSensor rgb;
	boolean enabled = false;
	double rightbaseposition = 0;
	double leftbaseposition = 0;
    double rightmidposition = 0;
    double leftmidpostition = 0;
	double servoSpeed = 0.01;
	double clipmin = 0;
	double clipmax = 1;
	double eyesight;
	int calls = 0;


    double servoposition = .75;

	/**
	 * Constructor
	 */
	public RedDriveFinalSensor() {

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
		touch = hardwareMap.touchSensor.get("touch");
		eyes = hardwareMap.opticalDistanceSensor.get("eyes");
		rgb =hardwareMap.colorSensor.get("rgb");
        leftmidservo.setDirection(Servo.Direction.REVERSE);
		rightbaseservo.setDirection(Servo.Direction.REVERSE);
		calls=0;





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
		double throttle = -gamepad1.left_stick_y;
		double direction = gamepad1.right_stick_x;
		double right = throttle - direction;
		double left = throttle + direction;
eyesight = eyes.getLightDetected();

		telemetry.addData("dsl", eyesight);
		int red = rgb.red();
		int blue = rgb.blue();
		int green = rgb.green();
		telemetry.addData("Red value: ", red);
		telemetry.addData("Blue value: ", blue);
		telemetry.addData("Green value: ", green);
		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = scaleInput(right);
		left = scaleInput(left);
		if (touch.isPressed())
		{telemetry.addData("Button is pressed.",calls); enabled = true;}
		else telemetry.addData("Button is not pressed.",calls);

		// write the values to the motors
		motorFrontRight.setPower(right);
		motorFrontLeft.setPower(left);
		motorBackLeft.setPower(left);
		motorBackRight.setPower(right);




		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

		calls++;

		// update the position of the arm.
		if (gamepad2.a) {
			// if the A button is pushed on gamepad1, increment the position of
			// the arm servo.
			rightbaseposition += 0.01;
			rightbaseposition = Range.clip(rightbaseposition, 0.0, 1);
			rightbaseservo.setPosition(rightbaseposition);

			//value when servo_1 is vertical--0.708
			//value when servo_1 is parallel to the ground--0.15
			leftbaseposition += 0.01;
			leftbaseposition = Range.clip(leftbaseposition, 0.0, 1);
			leftbaseservo.setPosition(leftbaseposition);


		}telemetry.addData("setting servo rightbase to " + rightbaseposition, calls);
		telemetry.addData("setting servo leftbase to " + leftbaseposition, calls);
		if (gamepad2.x) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the arm servo.
			leftbaseposition -= servoSpeed;
			leftbaseposition = Range.clip(leftbaseposition, clipmin	, clipmax);
			leftbaseservo.setPosition(leftbaseposition);

			//leftposition += 0.001;
			//leftposition = Range.clip(leftposition, 0.0, 1);
			//left.setPosition(leftposition);
			//telemetry.addData("setting servo_6, left servo to " + leftposition, calls);
			//value when servo_6 is completely vertical--0.737
			//value when servo_6 is parallel to the ground--0.137
			rightbaseposition -= servoSpeed;
			rightbaseposition = Range.clip(rightbaseposition, clipmin, clipmax);
			rightbaseservo.setPosition(rightbaseposition);


		}
        if (gamepad2.b) {
            // if the A button is pushed on gamepad1, increment the position of
            // the arm servo.
            rightmidposition += servoSpeed;
            rightmidposition = Range.clip(rightmidposition, clipmin, clipmax);
            rightmidservo.setPosition(rightmidposition);

            //value when servo_1 is vertical--0.708
            //value when servo_1 is parallel to the ground--0.15
            leftmidpostition += servoSpeed;
            leftmidpostition = Range.clip(leftmidpostition, clipmin, clipmax);
            leftmidservo.setPosition(leftmidpostition);


        }

        if (gamepad2.y) {
            // if the Y button is pushed on gamepad1, decrease the position of
            // the arm servo.
            leftmidpostition -= servoSpeed;
            leftmidpostition = Range.clip(leftmidpostition, clipmin, clipmax);
            leftmidservo.setPosition(leftmidpostition);

            //leftposition += 0.001;
            //leftposition = Range.clip(leftposition, 0.1, 0.99);
            //left.setPosition(leftposition);
            //telemetry.addData("setting servo_6, left servo to " + leftposition, calls);
            //value when servo_6 is completely vertical--0.737
            //value when servo_6 is parallel to the ground--0.137
            rightmidposition -= servoSpeed;
            rightmidposition = Range.clip(rightmidposition, clipmin, clipmax);
            rightmidservo.setPosition(rightmidposition);


        }

        if (gamepad2.left_bumper) {
            // if the A button is pushed on gamepad1, increment the position of
            // the arm servo.
            servoposition += 0.001;
            servoposition = Range.clip(servoposition, 0.6, 0.95);
            gripper.setPosition(servoposition);

            //value when servo_1 is vertical--0.708
            //value when servo_1 is parallel to the ground--0.15


        }

        if (gamepad2.right_bumper) {
            // if the Y button is pushed on gamepad1, decrease the position of
            // the arm servo.
            servoposition -= 0.001;
            servoposition = Range.clip(servoposition, 0.6, 0.95);
            gripper.setPosition(servoposition);}

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
