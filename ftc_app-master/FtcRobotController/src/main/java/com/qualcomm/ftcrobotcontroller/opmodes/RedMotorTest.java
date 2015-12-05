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

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class RedMotorTest extends OpMode {

	/*
	 * Tests motor function
	 * Pressing a,b,x,y on the gamepad moves the motors individually
	 */


	DcMotor motorFrontRight;
	DcMotor motorFrontLeft;
	DcMotor motorBackRight;
	DcMotor motorBackLeft;
    //DcMotor motor1;
    //DcMotor motor2;
    //DcMotor motor3;
    //DcMotor motor4;
    /*
     *motor2 is frontleft, moving backwards
     * motor4 is backleft, moving backwards
     * motor1 is backright, moving forwards
     * motor3 is frontright, moving forwards
     */



	/**
	 * Constructor
	 */
	public RedMotorTest() {

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

        telemetry.addData("hi", " here");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_2");
        motorFrontRight = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        motorBackRight = hardwareMap.dcMotor.get("motor_1");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        /*
     *motor2 is frontleft, moving backwards
     * motor4 is backleft, moving backwards
     * motor1 is backright, moving forwards
     * motor3 is frontright, moving forwards
     */



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



		// update the position of the arm.
		if (gamepad1.a) {
			// if the A button is pushed on gamepad1, increment the position of
			// the arm servo.
            telemetry.addData("moving", " motorFrontRight");
            motorFrontRight.setPower(1.0);
			motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            //motor1.setPower(1);
            //motor2.setPower(0);
            //motor3.setPower(0);
            //motor4.setPower(0);



		}

		if (gamepad1.y) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the arm servo.
            telemetry.addData("moving", " motorFrontLeft");
           motorFrontRight.setPower(0);
            motorFrontLeft.setPower(1);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
           // motor1.setPower(0);
            //motor2.setPower(1);
            //motor3.setPower(0);
            //motor4.setPower(0);
		}

		// update the position of the claw
		if (gamepad1.x) {
            telemetry.addData("moving", " motorBackLeft" );
           motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(1.0);
            motorBackRight.setPower(0);
           // motor1.setPower(0);
           // motor2.setPower(0);
           // motor3.setPower(1);
           // motor4.setPower(0);
		}

		if (gamepad1.b) {
            telemetry.addData("moving", " motorBackRight" );
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(1);
            //motor1.setPower(0);
            //motor2.setPower(0);
            //motor3.setPower(0);
            //motor4.setPower(1);
		}



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
