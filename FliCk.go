package main

import (
	"ROBOT/bot"
	"fmt"
	"os"
	"strconv"
	"time"
)

const arrayLenght = 50

// Cell is one slot of the shift array
type Cell struct {
	mediaE   float64
	deltaTxE float64
	deltaT   int64
}

// In struct contains inputs from board
type In struct {
	lWheelEnc int
	rWheelEnc int
	lLegEnc   int
	rLegEnc   int
	xGyro     float64
	yGyro     float64
	zGyro     float64
	time      time.Time
	deltaT    int64
}

// Out struct contains commands to the board
type Out struct {
	lWheelMotor int
	rWheelMotor int
	lLegMotor   int
	rLegMotor   int
}

// PidINFO struct contains vars of PID and time-window calculation
type PidINFO struct {
	windowTime   int64
	windowLenght int
	firstI       int
	currentI     int
	shift        [arrayLenght]Cell
	e            float64
	i            float64
	d            float64
	mediaE       float64
	deltaTxE     float64
	deltaTxEsum  float64
	deltaT       int64
	deltaTsum    int64
	correction   float64
	kP           float64
	kI           float64
	kD           float64
}

var in In
var out Out

// newPidINFO creates a new PidINFO struct
func newPidINFO(windowTime int64, kP float64, kI float64, kD float64) PidINFO {
	var n PidINFO

	n.windowTime = windowTime
	n.kP = kP
	n.kI = kI
	n.kD = kD

	return n
}

// addCell adds a Cell to the window (PidINFO)
func (pidINFO *PidINFO) addCell() {
	pidINFO.currentI++
	pidINFO.currentI = pidINFO.currentI % arrayLenght

	pidINFO.windowLenght++

	pidINFO.shift[pidINFO.currentI].deltaT = pidINFO.deltaT
	pidINFO.shift[pidINFO.currentI].deltaTxE = float64(pidINFO.deltaT) * pidINFO.e

	pidINFO.deltaTsum = pidINFO.deltaTsum + pidINFO.shift[pidINFO.currentI].deltaT
	pidINFO.deltaTxEsum = pidINFO.deltaTxEsum + pidINFO.shift[pidINFO.currentI].deltaTxE
}

// rmvCell removes a Cell from the window (PidINFO)
func (pidINFO *PidINFO) rmvCell() {
	pidINFO.deltaTsum = pidINFO.deltaTsum - pidINFO.shift[pidINFO.firstI].deltaT
	pidINFO.deltaTxEsum = pidINFO.deltaTxEsum - pidINFO.shift[pidINFO.firstI].deltaTxE

	pidINFO.shift[pidINFO.firstI].deltaT = 0
	pidINFO.shift[pidINFO.firstI].deltaTxE = 0
	pidINFO.shift[pidINFO.firstI].mediaE = 0

	pidINFO.firstI++
	pidINFO.firstI = pidINFO.firstI % arrayLenght

	pidINFO.windowLenght = pidINFO.windowLenght - 1
}

// computePid calculates P,I,D and manages the time-window (PidINFO, In)
func (pidINFO *PidINFO) computePid(e float64, deltaT int64) {
	pidINFO.e = e
	pidINFO.deltaT = deltaT

	if pidINFO.windowLenght >= arrayLenght {
		pidINFO.rmvCell()
	}

	pidINFO.addCell()

	for pidINFO.deltaTsum > pidINFO.windowTime && pidINFO.windowLenght > 1 {
		pidINFO.rmvCell()
	}

	pidINFO.mediaE = pidINFO.deltaTxEsum / float64(pidINFO.deltaTsum)
	pidINFO.shift[pidINFO.currentI].mediaE = pidINFO.mediaE

	pidINFO.i = pidINFO.i + (pidINFO.deltaTxEsum / 1000000)

	// anti wind-up
	if pidINFO.i > 1000000 {
		pidINFO.i = 1000000
	} else if pidINFO.i < -1000000 {
		pidINFO.i = -1000000
	}

	pidINFO.d = (pidINFO.shift[pidINFO.currentI].mediaE - pidINFO.shift[pidINFO.firstI].mediaE) / (float64(pidINFO.deltaTsum) / 1000000)

	pidINFO.correction = (pidINFO.kP * pidINFO.mediaE) + (pidINFO.kI * pidINFO.i) + (pidINFO.kD * pidINFO.d)
}

// reader reads data from the board (In)
func reader() {
	in.xGyro, in.yGyro, in.zGyro = bot.ReadGyro()
	in.lWheelEnc, in.rWheelEnc, in.rLegEnc, in.lLegEnc = bot.ReadAllEncoders()

	// deltaT in microseconds
	in.deltaT = time.Since(in.time).Nanoseconds() / 1000
	in.time = time.Now()
}

// setWheelPwr sets the power to wheels' motors (Out, PidINFO)
func (pidINFO *PidINFO) setWheelsPwr() {
	out.lWheelMotor = int(pidINFO.correction)
	out.rWheelMotor = int(pidINFO.correction)
}

// applyMotorsCmd sends the command to the motors (Out)
func applyMotorsCmd() {
	bot.SetMotor(1, out.lWheelMotor)
	bot.SetMotor(2, out.rWheelMotor)
	// bot.SetMotor(4, -out.lLegMotor)
	// bot.SetMotor(3, -out.rLegMotor)
}

// stopwheels sends the STOP command to the wheels' motors
func stopwheels() {
	bot.SetMotor(1, 0)
	bot.SetMotor(2, 0)
}

func main() {
	bot.Init()
	defer bot.Close()

	// parameters: arrayLenght, windowTime, kP, kI, kD ...1, 0.5, 0.01
	gyroPid := newPidINFO(5000, 0, 0, 0)

	// TO CREATE VARS...
	var i = 0
	var infoT [5000]int64
	var infoE [5000]float64
	var infoI [5000]float64
	var infoD [5000]float64
	var infoDeltaT [5000]int64
	var infoWindowSlots [5000]int
	var infoPwr [5000]int

	in.time = time.Now() // NOT to create vars

	var start = in.time
	//.

	for {
		if bot.CheckQuit() {
			stopwheels()
			break
		}

		reader()
		gyroPid.computePid(-in.yGyro, in.deltaT)

		// TO GET VALUES...
		infoT[i] = time.Since(start).Nanoseconds() / 1000

		infoE[i] = gyroPid.mediaE
		infoI[i] = gyroPid.i
		infoD[i] = gyroPid.d
		infoDeltaT[i] = gyroPid.deltaT
		infoWindowSlots[i] = gyroPid.windowLenght
		infoPwr[i] = out.lWheelMotor

		i++
		if i == 5000 {
			break
		}
		//i = i % 5000
		//.

		// gyroPid.setWheelsPwr()
		// applyMotorsCmd()
	}

	// TO WRITE THE FILE...
	file, err := os.Create("test.txt")
	if err != nil {
		fmt.Println(err)
		return
	}

	i = 0

	for {
		s := strconv.FormatInt(infoT[i], 10) + " " + strconv.FormatFloat(infoE[i], 'f', -1, 64) + " " + strconv.FormatFloat(infoI[i], 'f', -1, 64) + " " + strconv.FormatFloat(infoD[i], 'f', -1, 64) + " " + strconv.FormatInt(int64(infoDeltaT[i]), 10) + " " + strconv.FormatInt(int64(infoWindowSlots[i]), 10) + " " + strconv.FormatInt(int64(infoPwr[i]), 10) + "\n"
		_, err := file.WriteString(s)
		if err != nil {
			fmt.Println(err)
			file.Close()
			return
		}

		i++
		if i >= 5000 {
			fmt.Println(i)
			file.Close()
			break
		}
	}
	//.
}
