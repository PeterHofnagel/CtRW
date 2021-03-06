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
public class CTRW_speedtest_v1 extends OpMode {

	/*
	 *
	 */
    // Declaring motors and servos

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    int calls = 0;
    int[] forwardArray = {150, 200, 350, 500};
    int pressCount = 0;
    boolean motorRunning = false;
    int[] turnArray = {40, 60, 80, 100, 125, 150, 180, 200};
    boolean inAutonomous = false;
    boolean inForwardMode = false;
    boolean inTurnMode = false;




    /**
     * Constructor
     */
    public CTRW_speedtest_v1() {

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
        calls=0;




    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        handleForwardTest();
        handleTurnTest();
        pseudoAutonomous();
        if (gamepad1.y) //shut everything down and reinitialize all variables
        {
            setForwardPower(0.0);
            calls = 0;
            pressCount = 0;
            motorRunning = false;
            inAutonomous = false;
            inTurnMode = false;
            inForwardMode = false;
        }
        calls++;
    }

    public void setForwardPower(double value)
    {

        motorFrontRight.setPower(value);
        motorFrontLeft.setPower(value);
        motorBackLeft.setPower(value);
        motorBackRight.setPower(value);
    }

    public void setTurnPower(double value)
    {
        motorFrontRight.setPower(value);
        motorFrontLeft.setPower(-value);
        motorBackLeft.setPower(-value);
        motorBackRight.setPower(value);
    }

    public void handleForwardTest()
    {
        int cyclesToRun = forwardArray[pressCount%(forwardArray.length)];
        if (!motorRunning && gamepad1.a)  //motor not running and button is pressed
        {
            inForwardMode = true;
            setForwardPower(1.0);
            calls = 0;
            motorRunning = true;
            telemetry.addData("Starting " + cyclesToRun, " cycles");
        }
        else if (motorRunning && (calls >= cyclesToRun) && inForwardMode)
        {
            setForwardPower(0.0);
            pressCount++;
            motorRunning = false;
            inForwardMode = false;
            telemetry.addData("Finished " + cyclesToRun, " cycles");
        }
    }

    public void handleTurnTest()
    {
        int cyclesToRun = turnArray[pressCount%(turnArray.length)];
        if (!motorRunning && gamepad1.b)  //motor not running and button is pressed
        {
            inTurnMode = true;
            setTurnPower(1.0);
            calls = 0;
            motorRunning = true;
            telemetry.addData("Starting " + cyclesToRun, " cycles");
        }
        else if (motorRunning && (calls >= cyclesToRun) && inTurnMode)
        {
            setTurnPower(0.0);
            pressCount++;
            motorRunning = false;
            inTurnMode = false;
            telemetry.addData("Finished " + cyclesToRun, " cycles");
        }
    }

    public void pseudoAutonomous()
    {
        if (!motorRunning && gamepad1.x)  //motor not running and button is pressed
        {
            inAutonomous = true;
            setForwardPower(1.0);
            calls = 0;
            motorRunning = true;
            telemetry.addData("Starting ", "pseudoAutonomous");
        }
        else if ((motorRunning && inAutonomous) && (calls > 200 && calls < 325))
        {
            setTurnPower(1.0);
        }
        else if ((motorRunning && inAutonomous) && (calls > 325))
        {
            setForwardPower(1.0);
            calls = 0;
        }
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
