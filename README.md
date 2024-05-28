# scioly

Code and designs from the 2024 Science Olympiad season, competing in Division C.

## Robot Tour

The task for this year's event was to navigate a robot through a course autonomously in a 4x4 grid (each square 50cm in length), avoiding placed barries and entering certain squares ("gate zones") to earn more points, all while completing the course in as close to a specified target time as possible and stopping the robot as close as possible to a specified target point within the grid.

The robot described below did very well in its competition.

### Hardware:

* Arduino Mega 2560 R3
* Arduino Mega sensor shield
* Metal chassis kit
* 12V DC encoder motors
* Motor drivers
* Mecanum wheels
* Right-angle motor mount brackets
* AA battery pack with barrel jack
* Gyroscope/accelerometer
* Dupont wires, assorted
* M2/M2.5/M3/M4 screws/nuts/washers, assorted
* M2/M3/M4 brass standoffs, assorted
* Wooden dowel
* Cable ties
* Electrical tape
* Masking tape

\*\**Some details withheld to protect robot design*\*\*

### Software:

* Arduino IDE
* [Robot code](/robot-tour/robot.ino) (C++)
* MPU6050 (library) by ElectronicCats

## Detector Building

This year, the task was to build an ORP/Redox probe to detect and report concentrations of NaCl in an solution, along with its corresponding voltage. The device also had to light one or more of three LEDs depending on the concentration it had calculated.

The probe described below did alright in its competition, but results were inconsistent and should not be relied upon. However, the LED concentration code may be worth reuse.

### Hardware:

* Arduino Uno R3
* Ward's Science Detector Building kit
* 0/1000/2000/3000/4000/5000 ppm NaCl test samples
* Category C eye protection
* Graduated cylinder
* Disposable gloves
* Disposable pipettes
* Containers

### Software:

* Arduino IDE
* [Detector code](/detector-building/detector.ino) (C++)

### Material measurements:

* Silver wire - 15 cm
* Copper wire - 10 cm
* Yarn - 9 cm
* Plastic tubing - 10 cm
