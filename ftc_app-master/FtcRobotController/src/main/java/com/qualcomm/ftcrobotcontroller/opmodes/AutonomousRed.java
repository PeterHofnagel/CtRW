//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.Direction;

public class AutonomousRed extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    Servo rightbaseservo;
    Servo leftbaseservo;
    Servo rightmidservo;
    Servo leftmidservo;
    Servo gripper;
    ColorSensor rgb;
    OpticalDistanceSensor eyes;
    double rightbaseposition = 0.0D;
    double leftbaseposition = 0.0D;
    double rightmidposition = 0.0D;
    double leftmidpostition = 0.0D;
    double gripperposition = 0.0D;
    double servoSpeed = 0.01D;
    double clipmin = 0.0D;
    double clipmax = 1.0D;
    int calls = 0;
    double right = 0.0D;
    double left = 0.0D;
    double servoposition = 0.75D;
    double distance;
    public void SetPower(){
        this.motorFrontRight.setPower(right);
        this.motorFrontLeft.setPower(left);
        this.motorBackLeft.setPower(left);
        this.motorBackRight.setPower(right);
    }
    public void SetPosition(){
        this.rightbaseservo.setPosition(this.rightbaseposition);
        this.leftbaseservo.setPosition(this.leftbaseposition);
        this.rightmidservo.setPosition(this.rightmidposition);
        this.leftmidservo.setPosition(this.leftmidpostition);
        this.gripper.setPosition(this.gripperposition);
    }

    public AutonomousRed() {
    }

    public void init() {
        this.motorFrontLeft = (DcMotor)this.hardwareMap.dcMotor.get("frontleft");
        this.motorFrontRight = (DcMotor)this.hardwareMap.dcMotor.get("frontright");
        this.motorBackLeft = (DcMotor)this.hardwareMap.dcMotor.get("backleft");
        this.motorBackRight = (DcMotor)this.hardwareMap.dcMotor.get("backright");
        this.motorFrontRight.setDirection(Direction.REVERSE);
        this.motorBackRight.setDirection(Direction.REVERSE);
        this.rightbaseservo = (Servo)this.hardwareMap.servo.get("rightbase");
        this.leftbaseservo = (Servo)this.hardwareMap.servo.get("leftbase");
        this.rightmidservo = (Servo)this.hardwareMap.servo.get("rightmid");
        this.leftmidservo = (Servo)this.hardwareMap.servo.get("leftmid");
        this.gripper = (Servo)this.hardwareMap.servo.get("gripper");
        this.leftmidservo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        this.rightbaseservo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        this.calls = 0;
        this.rgb = (ColorSensor)this.hardwareMap.colorSensor.get("rgb");
        this.eyes = (OpticalDistanceSensor)this.hardwareMap.opticalDistanceSensor.get("eyes");
    }

    public void loop() {

        int red = this.rgb.red();
        int green = this.rgb.green();
        int blue = this.rgb.blue();
        distance = this.eyes.getLightDetected();
        SetPower();
        ++this.calls;
        SetPosition();
        if (this.distance > 0)
        {
            right = 0; left = 0; SetPower(); calls = 0;
        }
        if(this.calls > 350) {
            right = 0.0D;
            left = 0.0D;
            SetPower();
            this.rightbaseposition = 0.99D;
            this.leftbaseposition = 0.99D;
            this.rightmidposition = 0.99D;
            this.leftmidpostition = 0.99D;
            SetPosition();
            if(red > green * blue && red != 0 && this.calls < 500) {
                right = .50D;
                left = .5;
                SetPower();
            }
            else {right = 0; left = 0;}
        }

    }

    public void stop() {
    }

    double scaleInput(double dVal) {
        double[] scaleArray = new double[]{0.0D, 0.05D, 0.09D, 0.1D, 0.12D, 0.15D, 0.18D, 0.24D, 0.3D, 0.36D, 0.43D, 0.5D, 0.6D, 0.72D, 0.85D, 1.0D, 1.0D};
        int index = (int)(dVal * 16.0D);
        if(index < 0) {
            index = -index;
        }

        if(index > 16) {
            index = 16;
        }

        double dScale = 0.0D;
        if(dVal < 0.0D) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
