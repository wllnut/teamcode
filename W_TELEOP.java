package org.firstinspires.ftc.teamcode;

/* --------------------------- Robot Dependencies --------------------------- */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* ---------------------------------- Utils --------------------------------- */

import java.util.function.Supplier;
import java.util.HashMap;
import java.lang.Math;
import java.lang.InterruptedException;

/* -------------------------------------------------------------------------- */

@TeleOp
public class W_TELEOP extends LinearOpMode {

  /* -------------------------------------------------------------------------- */
  /*                                  SETTINGS                                  */
  /* -------------------------------------------------------------------------- */

  /* --------------------------------- DRIVING -------------------------------- */
  private final double STRAFE_ADJUSTMENT = 1.1; // Change in VERY SMALL increments to counteract imperfect strafing

  /* --------------------------------- Motors --------------------------------- */

  final String[] MOTORS_MOVEMENT_NAME_ARRAY = {"lBack", "rBack", "lFront", "rFront"};
  final String[] MOTORS_ARM_NAME_ARRAY = {"aRotate"};
  final String[] MOTORS_ETC_NAME_ARRAY = {};

  private final double ARM_MOTOR_POWER = 1.0;

  /* --------------------------------- Servos --------------------------------- */

  final String[] SERVOS_INTAKE_NAME_ARRAY = {"iRotate1", "iRotate2"};
  final String[] SERVOS_ETC_NAME_ARRAY = {};


  /* -------------------------------------------------------------------------- */
  /*                                 Declare IMU                                */
  /* -------------------------------------------------------------------------- */

  IMU imu;

  /* -------------------------------------------------------------------------- */
  /*                     Declare Motor and Servo HashMaps                       */
  /* -------------------------------------------------------------------------- */

  private HashMap<String, DcMotor> motors;
  private HashMap<String, Servo> servos;

  private DcMotor[] motors_movement = new DcMotor[MOTORS_MOVEMENT_NAME_ARRAY.length];
  private DcMotor[] motors_arm = new DcMotor[MOTORS_ARM_NAME_ARRAY.length];

  /* -------------------------------------------------------------------------- */
  /*                      Declare Gamepad-Related Variables                     */
  /* -------------------------------------------------------------------------- */

  private Gamepad gamepad1_current = new Gamepad();
  private Gamepad gamepad2_current = new Gamepad();

  private Gamepad gamepad1_previous = new Gamepad();
  private Gamepad gamepad2_previous = new Gamepad();

    /* -------------------------------------------------------------------------- */
    /*                                BUTTON STATE                                */
    /* -------------------------------------------------------------------------- */

    HashMap<GamepadButton, STATE> buttonStateMap = new HashMap<>();
    private void init_button_state_map() {
      buttonStateMap.put(new GamepadButton(() -> gamepad2_current.dpad_left, () -> !gamepad2_previous.dpad_left), STATE.INTAKE);
      buttonStateMap.put(new GamepadButton(() -> gamepad2_current.dpad_right, () -> !gamepad2_previous.dpad_right), STATE.DUMP);
    }

  /* -------------------------------------------------------------------------- */

  @Override
  public void runOpMode() throws InterruptedException {

    /* -------------------------------------------------------------------------- */
    /*                     Initialize Motor and Servo HashMaps                    */
    /* -------------------------------------------------------------------------- */

    motors = new HashMap<>();
    servos = new HashMap<>();

    /* -------------------------------------------------------------------------- */
    /*                Add Motors and Servos to HashMaps and Arrays                */
    /* -------------------------------------------------------------------------- */

    // DO NOT MANUALLY CHANGE THESE VALUES
  
    int MOTOR_MOVEMENT_QUANTITY = MOTORS_MOVEMENT_NAME_ARRAY.length;
    int MOTOR_ARM_QUANTITY = MOTORS_ARM_NAME_ARRAY.length;
    int MOTOR_ETC_QUANTITY = MOTORS_ETC_NAME_ARRAY.length;
    
    int SERVO_INTAKE_QUANTITY = SERVOS_INTAKE_NAME_ARRAY.length;
    int SERVO_ETC_QUANTITY = SERVOS_ETC_NAME_ARRAY.length;

    int __motor_quantity__ = Math.abs( ( MOTOR_MOVEMENT_QUANTITY + MOTOR_ARM_QUANTITY + MOTOR_ETC_QUANTITY ) );
    int __servo_quantity__ = Math.abs( (SERVO_INTAKE_QUANTITY + SERVO_ETC_QUANTITY) );

    /* ------------------------------ Intake Servos ----------------------------- */


    for (Integer i = 0; i < SERVO_INTAKE_QUANTITY; i++) {
      servos.put(SERVOS_INTAKE_NAME_ARRAY[i], hardwareMap.get(Servo.class, i.toString()));
    }

    /* ------------------------------ Other Servos ------------------------------ */

    for (Integer i = ( 1 + SERVO_INTAKE_QUANTITY ); i < __servo_quantity__; i++) {
      servos.put(SERVOS_ETC_NAME_ARRAY[i], hardwareMap.get(Servo.class, i.toString()));
    }

    /* ----------------------------- Movement Motors ---------------------------- */

    for (Integer i = 0; i < Math.abs(MOTOR_MOVEMENT_QUANTITY); i++) {
      if (i != 2) {
        motors.put(MOTORS_MOVEMENT_NAME_ARRAY[i], hardwareMap.get(DcMotor.class, i.toString()));
      }
    }

    for (int i = 0; i < MOTOR_MOVEMENT_QUANTITY; i++) {
      motors_movement[i] = motors.get(MOTORS_MOVEMENT_NAME_ARRAY[i]);
    }

    /* --------------------------- Arm Control Motors --------------------------- */

    motors.put(MOTORS_ARM_NAME_ARRAY[0], hardwareMap.get(DcMotor.class, "2"));
    motors_arm[0] = motors.get(MOTORS_ARM_NAME_ARRAY[0]);

    /* ------------------------------ Other Motors ------------------------------ */

    for (Integer i = ( __motor_quantity__ - ( MOTOR_MOVEMENT_QUANTITY + MOTOR_ARM_QUANTITY ) ); i < __motor_quantity__; i++) {
      motors.put(MOTORS_ETC_NAME_ARRAY[i], hardwareMap.get(DcMotor.class, i.toString()));
    }

    /* ----------------------------------- IMU ---------------------------------- */

    imu = hardwareMap.get(IMU.class, "imu");

    /* --------------- ADJUST PARAMETERS DEPENDING ON ROBOT BUILD --------------- */
    /* ---------------------- THESE ARE PLACEHOLDER VALUES ---------------------- */

    IMU.Parameters parameters = new IMU.Parameters(
        new RevHubOrientationOnRobot(
          RevHubOrientationOnRobot.LogoFacingDirection.UP,
          RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )
    );

    imu.initialize(parameters);

    /* ----------------------- Initialize Button State Map ---------------------- */
    init_button_state_map();

    /* -------------------------------------------------------------------------- */

    waitForStart();

    while (opModeIsActive()) {

    /* -------------------------------------------------------------------------- */
    /*                            Store Gamepad States                            */
    /* -------------------------------------------------------------------------- */

      /*
      * Store the gamepad values from the previous loop iteration in
      * gamepad[1/2]_current to be used in this loop iteration.
      * This is equivalent to doing this at the end of the previous loop iteration
      */

      gamepad1_previous.copy(gamepad1_current);
      gamepad2_previous.copy(gamepad2_current);
      
      /* -------------------------------------------------------------------------- */
      
      /*
      * Store the gamepad values from this loop iteration in
      * gamepad[1/2]_current to be used for this loop iteration
      */

      gamepad1_current.copy(gamepad1);
      gamepad2_current.copy(gamepad2);

      /* -------------------------------------------------------------------------- */
      /*                            RUN MAIN TELEOP LOOP                            */
      /* -------------------------------------------------------------------------- */

      main_loop();
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                      ARM CONTROL FINITE STATE MACHINE                      */
  /* -------------------------------------------------------------------------- */

  /* ------------------------------- State Enum ------------------------------- */

  private enum STATE {
    START,
    INTAKE,
    CLOSE,
  }

  private enum SERVO_STATE {
    START,
    OPEN,
    CLOSE,
  }

  // Declare State for future use in main_loop()
  private STATE state = STATE.START;
  private SERVO_STATE servo_state = SERVO_STATE.START;

  /* -------------------------------------------------------------------------- */
  /*                               STATE FUNCTIONS                              */
  /* -------------------------------------------------------------------------- */

  private final void state_reset() {
    state = STATE.START;
    servo_state = SERVO_STATE.START;
  }

  private final void state_set(STATE s) {
    state = s;
  }

  private STATE check_gamepad_input() {
    motors_arm[0].setPower(gamepad2.right_stick_y);
    for (GamepadButton button : buttonStateMap.keySet()) {
      if (button.isPressed()) {
        return buttonStateMap.get(button);
      }
    }
    return STATE.START;
  }

  /* -------------------------------------------------------------------------- */
  /*                                ARM FUNCTIONS                               */
  /* -------------------------------------------------------------------------- */

  private final void arm_motors_move(DcMotorSimple.Direction dir) {
    for (int i = 0; i < MOTORS_ARM_QUANTITY; i++) {
      motors_arm[i].setPower( (dir == DcMotorSimple.Direction.FORWARD ? Math.abs(ARM_MOTOR_POWER) : -Math.abs(ARM_MOTOR_POWER) ) );
    }
  }

  private final void servo_update() {
    servos.get(SERVOS_INTAKE_NAME_ARRAY[0]).setPosition(gamepad2.right_stick_y);
    servos.get(SERVOS_INTAKE_NAME_ARRAY[1]).setPosition(gamepad2.right_stick_y);
  }

  /* -------------------------------------------------------------------------- */
  /*                         FIELD CENTRIC MECANUM DRIVE                        */
  /* -------------------------------------------------------------------------- */

  private final void update_drive() {

    /* -------------------- Get Gamepad1 Analog Stick Values -------------------- */

    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
    double x = gamepad1.left_stick_x;
    double rx = gamepad1.right_stick_x;

    /* ------------------------  BACK to reset robot yaw ------------------------ */

    if(gamepad1.back && !gamepad1_previous.back) {
      imu.resetYaw();
    }

    /* ------------------------ Get Current Robot Heading ----------------------- */

    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    /* ----------- Rotate Movement Direction Counter to Robot Rotation ---------- */

    double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * STRAFE_ADJUSTMENT;
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


    /* ------------------------ Power Mecanum Kinematics ------------------------ */

    double denominator = Math.max( Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1 );
    double lFrontPower = (rotY + rotX + rx) / denominator;
    double lBackPower = (rotY - rotX + rx) / denominator;
    double rFrontPower = (rotY - rotX - rx) / denominator;
    double rBackPower = (rotY + rotX - rx) / denominator;

    motors_movement[0].setPower(lBackPower);
    motors_movement[1].setPower(rBackPower);
    motors_movement[2].setPower(lFrontPower);
    motors_movement[3].setPower(rFrontPower);


  }

  /* -------------------------------------------------------------------------- */
  /*                                  MAIN LOOP                                 */
  /* -------------------------------------------------------------------------- */

  private final void main_loop() {
    
    /* -------------------------------------------------------------------------- */
    /*                            FINITE STATE MACHINE                            */
    /* -------------------------------------------------------------------------- */

    switch (state) {

      case START:
        state_set(check_gamepad_input());
        break;

      case INTAKE:
        servo_state = (servo_state != SERVO_STATE.OPEN ? SERVO_STATE.OPEN : servo_state);
        break;
      
      case CLOSE:
        servo_state = (servo_state != SERVO_STATE.CLOSE ? SERVO_STATE.CLOSE : servo_state);
        break;
    }

    if (gamepad2.a) {
      state_reset();
    }

    /* -------------------------------------------------------------------------- */
    /*                          UPDATE MECANUM DRIVETRAIN                         */
    /* -------------------------------------------------------------------------- */
    servo_update();
    update_drive();

  }
}

/* -------------------------------------------------------------------------- */
/*                          BUTTON LOGIC HELPER CLASS                         */
/* -------------------------------------------------------------------------- */

class GamepadButton {
  private final Supplier<Boolean> isCurrentPressed;
  private final Supplier<Boolean> isPreviousNotPressed;

  public GamepadButton(Supplier<Boolean> isCurrentPressed, Supplier<Boolean> isPreviousNotPressed) {
      this.isCurrentPressed = isCurrentPressed;
      this.isPreviousNotPressed = isPreviousNotPressed;
  }

  public boolean isPressed() {
      return (isCurrentPressed.get() && isPreviousNotPressed.get());
  }
}
