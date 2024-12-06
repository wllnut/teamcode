package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// Utils
import java.util.HashMap;

@TeleOp
public class W_TELEOP extends LinearOpMode {

  // Lift Initialization
  private enum LIFT_STATE {
    LIFT_START,
    LIFT_ROTATE_UP,
    LIFT_ROTATE_DOWN,
    LIFT_INTAKE,
    LIFT_DUMP,
    LIFT_RETURN,
  };

  LIFT_STATE lift_state = LIFT_STATE.LIFT_START;

  ElapsedTime lift_timer = new ElapsedTime();

  // Requires manual calibration
  final double LIFT_EXTEND_TIME = 3;
  final double LIFT_RETRACT_TIME = 3;
  final double LIFT_DUMP_TIME = 1.5;

  // Motor and Servo Initialization
  private HashMap<String, DcMotor> motors;
  private DcMotor[] motors_arm = new DcMotor[2];
  private DcMotor[] motors_movement = new DcMotor[4];
  private DcMotor motors_aRotate1;
  private DcMotor motors_aRotate2;

  private HashMap<String, Servo> servos;
  private Servo servo_lift_1;
  private Servo servo_lift_2;

  // Gamepad Initialization
  @Override
  public void runOpMode() {
    // Initialize motor and servo arrays

    // Setup Servo HashMap
    servos = new HashMap<>();
    
    // Setup Motor HashMap
    motors = new HashMap<>();
    
    // Intake Rotation
    servos.put("iRotate1", hardwareMap.get(Servo.class, "s1"));
    servos.put("iRotate2", hardwareMap.get(Servo.class, "s2"));
    
    // Movement
    motors.put("lBack", hardwareMap.get(DcMotor.class, "0"));
    motors.put("rBack", hardwareMap.get(DcMotor.class, "1"));

    // Arm Control
    motors.put("aRotate1", hardwareMap.get(DcMotor.class, "2"));
    motors.put("aRotate2", hardwareMap.get(DcMotor.class, "3"));

    motors.get("lBack").setDirection(DcMotorSimple.Direction.REVERSE); // Modify or add lines if motor setup changes

    motors_movement[0] = motors.get("lBack");
    motors_movement[1] = motors.get("rBack");
    motors_arm[0] = motors.get("aRotate1");
    motors_arm[1] = motors.get("aRotate2");
    motors_aRotate1 = motors.get("aRotate1");
    motors_aRotate2 = motors.get("aRotate2");
    servo_lift_1 = servos.get("iRotate1");
    servo_lift_2 = servos.get("iRotate2");
    
    
    waitForStart();

    while (opModeIsActive()) {
      main_loop();
    }
  }
  
  public void main_loop() {
    // Arm Control Finite State Machine
    switch (lift_state) {
      case LIFT_START:
        lift_timer.reset();

        lift_state_set(lift_state_await_input());

        break;
        
      case LIFT_ROTATE_UP:
        if (lift_is_extended()) {
          motors_aRotate1.setPower(0.0);
          motors_aRotate2.setPower(0.0);
          lift_state_reset();
          break;
        }
        arm_rotate(DcMotorSimple.Direction.FORWARD);
        servo_lift_1.setPosition(1.0);
        servo_lift_2.setPosition(1.0);
        break;

      case LIFT_ROTATE_DOWN:
        if (lift_is_retracted()) {
          motors_aRotate1.setPower(0.0);
          motors_aRotate2.setPower(0.0);
          lift_state_reset();
          break;
        }
        arm_rotate(DcMotorSimple.Direction.REVERSE);
        break;

      case LIFT_INTAKE:
        servo_lift_1.setPosition(1.0);
        servo_lift_2.setPosition(1.0);
        lift_state_reset();
        break;

      case LIFT_DUMP:
        if (lift_has_dumped()) {
          servo_lift_1.setPosition(0.0);
          servo_lift_2.setPosition(0.0);
          lift_state_reset();
          break;
        }
        servo_lift_1.setPosition(-1.0);
        servo_lift_2.setPosition(-1.0);
        break;

      case LIFT_RETURN:
        motors.forEach((key, value) -> {
          value.setPower(0.0);
        });

        servos.forEach((key, value) -> {
          value.setPosition(0.0);
        });
        
        lift_state_reset();

        break;

      default:
        // Should never be reached
        lift_state_reset();
        break;
    }
    if (gamepad2.back && lift_state != LIFT_STATE.LIFT_START) {
      lift_state_set(LIFT_STATE.LIFT_RETURN);
    }

    updateDrive();
  }

  private void updateDrive() {
    // REPLACE IF CHANGED TO MECANUM
    double analog_stick_left_y = gamepad1.left_stick_y;
    double analog_stick_right_y = gamepad1.right_stick_y;

    // motors_movement[0].setPower(analog_stick_left_y); // Left Front
    // motors_movement[1].setPower(analog_stick_right_y); // Right Front
    motors_movement[0].setPower(analog_stick_left_y); // Left Back
    motors_movement[1].setPower(analog_stick_right_y); // Right Back
  }

  private boolean lift_is_extended() {
    return (lift_timer.seconds() >= LIFT_EXTEND_TIME);
  }

  private boolean lift_is_retracted() {
    return (lift_timer.seconds() >= LIFT_RETRACT_TIME);
  }

  private boolean lift_has_dumped() {
    return (lift_timer.seconds() >= LIFT_DUMP_TIME);
  }

  private void lift_state_reset() {
    lift_state = LIFT_STATE.LIFT_START;
  }

  private void lift_state_set(LIFT_STATE state) {
    lift_state = state;
  }

  private LIFT_STATE lift_state_await_input() {
    if (gamepad2.dpad_down) {
      return LIFT_STATE.LIFT_ROTATE_DOWN;
    } else if (gamepad2.dpad_up) {
      return LIFT_STATE.LIFT_ROTATE_UP;
    } else if (gamepad2.x) {
      return LIFT_STATE.LIFT_INTAKE;
    } else if (gamepad2.a) {
      return LIFT_STATE.LIFT_DUMP;
    } else if (gamepad2.start) {
      return LIFT_STATE.LIFT_RETURN;
    } else {
      return LIFT_STATE.LIFT_START;
    }
  }

  
  private void arm_rotate(DcMotorSimple.Direction dir) {
     for (int i = 0; i < motors_arm.length; i++) {
        motors_arm[i].setDirection(dir);
        motors_arm[i].setPower(1.0);
       }
  }

}
