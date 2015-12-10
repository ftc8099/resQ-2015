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
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Robot8099TeleOp extends OpMode {

	/* extreme values for servo ranges */
	final static double SERVO_1_MIN = 0.20;
	final static double SERVO_1_MAX = 0.90;

	// position of servos 1 and 2
	double servo_1_position;

	// change in servo positions
	double servo_1_delta = 0.05;

	double motor_3_delta = 0.1;

	// create two servos
	Servo servo_1;

	// create two dc motors
	DcMotor motor_1;
	DcMotor motor_2;
	DcMotor motor_3;

	// create the sensors
	//IrSeekerSensor ir_seeker;
	OpticalDistanceSensor ods_sensor;
	TouchSensor touch_sensor;

	/**
	 * Constructor
	 */
	public Robot8099TeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		// acquire motors and servos from hardware
		motor_1 = hardwareMap.dcMotor.get("motor_1");
		motor_2 = hardwareMap.dcMotor.get("motor_2");
		motor_3 = hardwareMap.dcMotor.get("motor_3");
		servo_1 = hardwareMap.servo.get("servo_1");

		// get the sensors
		//ir_seeker = hardwareMap.irSeekerSensor.get("ir_seeker");
		ods_sensor = hardwareMap.opticalDistanceSensor.get("ods_sensor");
		touch_sensor = hardwareMap.touchSensor.get("touch_sensor");

		// reverse motor 2
		motor_2.setDirection(DcMotor.Direction.REVERSE);

		// initialize servo locations
		servo_1_position = SERVO_1_MIN;
	}


	// Get absolute value of double
	private double abs_d(double n){
		if(n >= 0) return n;
		else return (-1) * n;
	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		double motor_1_stick;
		double motor_2_stick;


		// ON CONTROLLER:
		// - no light next to mode button
		// - switch set to x on back
		// IN DRIVER APP
		// - controller set as Logitech F310 Gamepad
		//
		// y value: -1 = up, +1 = down
		// x value: -1 = left, +1 = right
		//

		// Joystick y inverted, so then the motor speed is
		// positive when pushing stick up
		double motor_speed = -gamepad1.left_stick_y;

		// Just get the plain stick x and use as the turn speed
		double turn_speed = gamepad1.right_stick_x;

		// Joystick is pushed forward or backward
		if(motor_speed != 0) {

			// If the turn speed is maxed at either 1 or -1,
			// this variable will cancel out the motor speed (by subtraction).
			// This is useful in turning (while moving) by preventing
			// motor from going the opposite direction and making things go crazy.
			double turn_speed_to_halt = abs_d(turn_speed) * motor_speed;

			// If joystick is push to the left (turn_speed < 0),
			// then motor 1 will slow down.
			motor_1_stick = motor_speed - ((turn_speed < 0) ? turn_speed_to_halt : 0);

			// If joystick is push to the right (turn_speed > 0),
			// then motor 2 will slow down.
			motor_2_stick = motor_speed - ((turn_speed > 0) ? turn_speed_to_halt : 0);
		}

		// Basic 'standing' turn
		else{
			motor_1_stick = -turn_speed;
			motor_2_stick = turn_speed;
		}

		// clip values so to be in [-1,1]
		motor_1_stick = Range.clip(motor_1_stick,-1,1);
		motor_2_stick = Range.clip(motor_2_stick,-1,1);
		// call motor response function
		motor_1_stick = (float)response_function(motor_1_stick);
		motor_2_stick = (float)response_function(motor_2_stick);

		// set motor values (inverted because of motor placement)
		motor_1.setPower(-motor_1_stick);
		motor_2.setPower(-motor_2_stick);


		if(touch_sensor.isPressed()){

			// Halt 3rd motor if touch sensor is touched
			motor_3.setPower(0);

		}else if(gamepad1.a) {

			// 3rd motor down when touch senor not pressed
			motor_3.setPower(motor_3_delta);
		}


		// 3rd motor up
		if (gamepad1.a) {
			motor_3.setPower(motor_3_delta);
		}

		// Servo left and right
		if(gamepad1.dpad_left) {
			servo_1_position -= servo_1_delta;
		}
		if(gamepad1.dpad_right) {
			servo_1_position += servo_1_delta;
		}

		// use a to ratchet down servo 2, x to ratchet down servo 1, y to ratchet up servo 2 and b to ratchet up servo 1
		/*if(gamepad1.left_bumper){
			servo_1_position-=servo_1_delta;
		}
		if(gamepad1.left_trigger>0){
			servo_1_position+=servo_1_delta;
		}*/

		// clip servos to prespectified range
		servo_1_position = Range.clip(servo_1_position,SERVO_1_MIN,SERVO_1_MAX);

		// set servo positions
		servo_1.setPosition(servo_1_position);

		/*if(touch_sensor.isPressed()) {
			motor_1.setPower(0.00);
			motor_2.setPower(0.00);
			motor_1_stick = 0.00;
			motor_2_stick = 0.00;
		}*/

		/*double ir_angle = 0.0;
		double ir_strength = 0.0;
		if (ir_seeker.signalDetected()) {
			/*
			 * Get angle and strength of the signal.
			 * Note an angle of zero implies straight ahead.
			 * A negative angle implies that the source is to the left.
			 * A positive angle implies that the source is to the right.
			 /
			ir_angle = ir_seeker.getAngle();
			ir_strength = ir_seeker.getStrength();
		}*/

		// write some telemetry to the driver station
	//	telemetry.addData("Text","*** Robot Data ***");
	//	telemetry.addData("servo_1","servo_1: "+String.format("%.2f",servo_1_position));
	//	telemetry.addData("servo_2","servo_2: "+String.format("%.2f",servo_2_position));
		telemetry.addData("motor_1","motor 1: "+String.format("%.2f",motor_1_stick));
		telemetry.addData("motor_2","motor 2: "+String.format("%.2f",motor_2_stick));
		telemetry.addData("turn_speed","turn_speed: "+String.format("%.2f",turn_speed));
		telemetry.addData("motor_speed","motor_speed: "+String.format("%.2f",motor_speed));
	//	telemetry.addData("ir detected","ir detected: "+String.format("%d",ir_seeker.signalDetected()));
	//	telemetry.addData("ir angle","ir angle: "+String.format("%.2f",ir_angle));
	//	telemetry.addData("ir strength","ir strength: "+String.format("%.2f",ir_strength));
	//	telemetry.addData("ods value","ods value: "+String.format("%.2f",light_detected));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

    	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double response_function(double dVal)  {
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
