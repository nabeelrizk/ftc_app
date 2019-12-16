package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2019 FIRST. All rights reserved.

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Basic: Iterative OpMode", group = "Iterative Opmode")
public class AutonomousOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    final static float rotationsPerInchForwardBackward = 0;
    final static float rotationsPerInchStrafing = 0;
    final static float rotationsPerDegreeRightLeft = 0;
    final static float maxPower = 0;
    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor spinRight;
    private DcMotor spinLeft;
    private Servo spool;
    private Servo grabServo;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        grabServo = hardwareMap.get(Servo.class, "arm");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        spinRight = hardwareMap.get(DcMotor.class, "spin_right");
        spinLeft = hardwareMap.get(DcMotor.class, "spin_left");
        spool = hardwareMap.get(Servo.class, "spool");
        spinLeft = hardwareMap.get(DcMotor.class, "spin_left");
        spinLeft = hardwareMap.get(DcMotor.class, "spin_right");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        waitForStart();

    }

    /*knowing the current position of the robot (where 0,0 is the corner of the field), you can pick up any block at a given angle.*/
    void pickUpBlock(int xPos, int yPos, int angle, int blockNum) {
        float targetXPos = blockNum * 8;
        float targetYPos = 30;
        float xNotMoved = (float) (Math.tan(Math.toRadians(angle) * (targetYPos - yPos)));
        float xMove = Math.abs(xPos - targetXPos) - xNotMoved;
        if (xPos < targetXPos) {
            strafe(xMove);
            turn(angle);
        } else {
            strafe(-xMove);
            turn(-angle);
        }
        feed(1);
        forwardBackward((float) (Math.sqrt((targetYPos - yPos) * (targetYPos - yPos) + xNotMoved * xNotMoved)));
        feed(0);
    }

    void feed(double power) {
        spinRight.setPower(power);
        spinLeft.setPower(power);
    }

    /*we can not push the platform as the other one will be in the way so I am assuming we
    will put a servo, however, I do not know where this mechanism will be. Because I do not know the orientation, this function is intentionally left blank.*/
    void grabPlatform(int x, int y, int orientation) {

    }

    void forwardBackward(float inches) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int) (inches * rotationsPerInchForwardBackward));

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int) (inches * rotationsPerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int) (inches * rotationsPerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int) (inches * rotationsPerInchForwardBackward));

        allOn();
        while (leftFront.isBusy() && opModeIsActive()) {
        }
        allOff();
    }

    void turn(float degrees) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int) (degrees * rotationsPerDegreeRightLeft));

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int) (degrees * rotationsPerDegreeRightLeft));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int) (-degrees * rotationsPerDegreeRightLeft));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int) (-degrees * rotationsPerDegreeRightLeft));

        allOn();
        while (leftFront.isBusy() && opModeIsActive()) {
        }
        allOff();
    }

    void strafe(float inches) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int) (inches * rotationsPerInchStrafing));

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int) (-inches * rotationsPerInchStrafing));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int) (inches * rotationsPerInchStrafing));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int) (-inches * rotationsPerInchStrafing));

        allOn();
        while (leftFront.isBusy() && opModeIsActive()) {
        }
        allOff();
    }

    void allOn() {
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(maxPower);
        rightFront.setPower(maxPower);
    }

    void allOff() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }


}

