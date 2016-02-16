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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 *
 * main Drive Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class CTRW_Autonomous_LineFollower_V2 extends OpMode {

	/*
	 *
	 */
    // Declaring motors and servos

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    long timer;

    boolean lineFound = false;
    boolean driveon = false;
    int calls = 0;
    int callswl = 0;
     OpticalDistanceSensor v_sensor_ods;
    /**
     * Constructor
     */
    public CTRW_Autonomous_LineFollower_V2() {

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
        motorFrontLeft = hardwareMap.dcMotor.get("frontleft"); //port1 left
        motorFrontRight = hardwareMap.dcMotor.get("frontright"); //port1 right
        motorBackLeft = hardwareMap.dcMotor.get("backleft"); //port2 left11a
        motorBackRight = hardwareMap.dcMotor.get("backright"); //port2 right
        set_drive_power(0, 0);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        v_sensor_ods = hardwareMap.opticalDistanceSensor.get("distanceSensor");
        v_sensor_ods.enableLed(true);


        //rightbaseservo = hardwareMap.servo.get("rightbase"); //port1
        //leftbaseservo = hardwareMap.servo.get("leftbase"); //port6
        //rightmidservo = hardwareMap.servo.get("rightmid"); //port3
        //leftmidservo = hardwareMap.servo.get("leftmid"); //port5
       // gripper = hardwareMap.servo.get("gripper"); //port2


        //rightmidservo.setDirection(Servo.Direction.REVERSE);
        //rightbaseservo.setDirection(Servo.Direction.REVERSE);
       // calls=0;
       // winch = hardwareMap.servo.get("winch");

       // winch.setPosition(0.5);



    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        calls++;
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

        if (!lineFound && calls <2200)
        {
            set_drive_power(.15,.15);
            if(a_ods_white_tape_detected())
            {
                lineFound = true;
            }
        }
        else if (calls >1200)
        {
            set_drive_power(0,0);
        }
        else
        {
            if (callswl >= 180)
            {
                set_drive_power(0,0);
            }
           else if (a_ods_white_tape_detected()) {
                set_drive_power(0.0, 0.4);
            }
            else {
                set_drive_power(0.4, 0.0);
            }
            callswl ++;
        }

        telemetry.addData("Light Value:", v_sensor_ods.getLightDetected());
        telemetry.addData("Calls:", calls);
        telemetry.addData("Status", v_sensor_ods.status());
        telemetry.addData("Left Power Front:", motorFrontLeft.getPower());
        telemetry.addData("Left Power Back:", motorBackLeft.getPower());
        telemetry.addData("Right Power Front:", motorFrontRight.getPower());
        telemetry.addData("Right Power Back:", motorBackRight.getPower());
        telemetry.addData("Calls on white line:", callswl);
    }

    boolean a_ods_white_tape_detected ()

    {
        //
        // Assume not.
        //
        boolean l_return = false;

        if (v_sensor_ods != null)
        {
            //
            // Is the amount of light detected above the threshold for white
            // tape?
            //
            if (v_sensor_ods.getLightDetected () > 0.15)
            {
                l_return = true;
            }
        }

        //
        // Return
        //
        return l_return;

    } // a_ods_white_tape_detected

    public void set_drive_power (double left, double right)
    {
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
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
