package bot

/*
#include <rc/led.h>
#include <rc/button.h>
#include <rc/time.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/motor.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder_pru.h>
#cgo LDFLAGS: -lrobotcontrol
*/
import "C"
import "fmt"

// Led enumeration
type Led int

// Definitions for sensors
var data C.rc_mpu_data_t
var conf C.rc_mpu_config_t = C.rc_mpu_default_config()

const (
	// Green led
	Green = iota
	// Red led
	Red
	// Usr0 led
	Usr0
	// Usr1 led
	Usr1
	// Usr2 led
	Usr2
	// Usr3 led
	Usr3
)

// Usleep sleeps for "time" microseconds
func Usleep(time uint) {
	C.rc_usleep(C.uint(time))
}

// SetLed sets the given led
func SetLed(led Led, value bool) {
	cValue := 0
	if value {
		cValue = 1
	}
	C.rc_led_set(C.rc_led_t(led), C.int(cValue))
}

// Init initialize the robot
func Init() {
	// button mode
	C.rc_button_init(2, 4, 1, 2000)
	// button pause
	C.rc_button_init(2, 5, 1, 2000)

	// init MPU
	conf.enable_magnetometer = 1
	C.rc_mpu_initialize(&data, conf)

	// init servos
	C.rc_servo_init()
	// enable servos power rail
	C.rc_servo_power_rail_en(1)

	// init motors
	C.rc_motor_init()

	// init encoders
	C.rc_encoder_eqep_init()
	C.rc_encoder_pru_init()

	// turn leds off
	LedsOff()
	ColorLedsOff()

	SetLed(Green, true)
	fmt.Println("initialized")
}

// Close prepare the robot to the shut down
func Close() {
	// disable MPU
	C.rc_mpu_power_off()

	// disable servos power rail
	C.rc_servo_power_rail_en(0)
	// disable servos
	C.rc_servo_cleanup()

	// disable motors
	C.rc_motor_cleanup()

	// stop motors
	StopMotors()

	// disable encoders
	C.rc_encoder_eqep_cleanup()
	C.rc_encoder_pru_cleanup()

	// turn leds off
	LedsOff()
	ColorLedsOff()

	fmt.Println("closed")
}

// ButtonModePressed read the mode button
func ButtonModePressed() bool {
	if C.rc_button_get_state(2, 4) == 1 {
		return true
	}
	return false
}

// ButtonPausePressed read the pause button
func ButtonPausePressed() bool {
	if C.rc_button_get_state(2, 5) == 1 {
		return true
	}
	return false
}

// ServoSendPulseUs send to the given servo a puls for the given usec
func ServoSendPulseUs(ch, us int) {
	C.rc_servo_send_pulse_us(C.int(ch), C.int(us))
}

// ServoSendPulsePos sets the given servo to the given position
func ServoSendPulsePos(ch, pos int) {
	// deg: -90   0   90
	// us:  600 1500 2400
	us := 1500 + (pos * 10)
	if us < 600 {
		us = 600
	} else if us > 2400 {
		us = 2400
	}
	ServoSendPulseUs(ch, us)
}

// SetMotor sets the given motor at duty cycle speed
func SetMotor(ch, speed int) {
	if speed < -100 {
		speed = -100
	} else if speed > 100 {
		speed = 100
	}
	duty := float64(speed) / 100
	C.rc_motor_set(C.int(ch), C.double(duty))
}

// StopMotors sets all motors to 0
func StopMotors() {
	SetMotor(1, 0)
	SetMotor(2, 0)
	SetMotor(3, 0)
	SetMotor(4, 0)
}

// ReadEncoder reads from the given encoder
func ReadEncoder(ch int) int {
	if ch == 4 {
		return int(C.rc_encoder_pru_read())
	}
	return int(C.rc_encoder_eqep_read(C.int(ch)))
}

// ReadAllEncoders reads from all the encoders
func ReadAllEncoders() (int, int, int, int) {
	return ReadEncoder(1), ReadEncoder(2), ReadEncoder(3), ReadEncoder(4)
}

// ResetAllEncoders resets all the encoders
func ResetAllEncoders() {
	C.rc_encoder_eqep_write(1, 0)
	C.rc_encoder_eqep_write(2, 0)
	C.rc_encoder_eqep_write(3, 0)
	C.rc_encoder_pru_write(0)
}

// ReadGyro reads from gyro (degrees/s)
func ReadGyro() (float64, float64, float64) {
	C.rc_mpu_read_gyro(&data)
	return float64(data.gyro[0]), float64(data.gyro[1]), float64(data.gyro[2])
}

// ReadAccel reads from accelerometer
func ReadAccel() (float64, float64, float64) {
	C.rc_mpu_read_accel(&data)
	return float64(data.accel[0]), float64(data.accel[1]), float64(data.accel[2])
}

// ReadMag reads from magnetometer
func ReadMag() (float64, float64, float64) {
	C.rc_mpu_read_mag(&data)
	return float64(data.mag[0]), float64(data.mag[1]), float64(data.mag[2])
}

// ReadTemp reads from termometer
func ReadTemp() float64 {
	C.rc_mpu_read_temp(&data)
	return float64(data.temp)
}

// LedsOff sets blue leds off
func LedsOff() {
	SetLed(Usr0, false)
	SetLed(Usr1, false)
	SetLed(Usr2, false)
	SetLed(Usr3, false)
}

// ColorLedsOff sets green and red leds off
func ColorLedsOff() {
	SetLed(Green, false)
	SetLed(Red, false)
}

// CheckQuit checks if you are trying to exit the program
func CheckQuit() bool {
	if ButtonPausePressed() {
		LedsOff()
		SetLed(Green, false)
		SetLed(Red, true)
		Usleep(1000000)
		return true
	}
	return false
}
