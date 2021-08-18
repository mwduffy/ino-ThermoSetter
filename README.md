# ino-ThermoSetter
Software for an Arduino-based replacement of a Masterbuilt Model 20070512 control unit 

This started as a helper app for calibrating thermistors contained in a Masterbuilt
smoker. The original intent was to set a target temperature and try to maintain that
temperature until the smoker box reaches equilibrium. At that point the power
was cut, the thermistor isolated and the resistances measured multiple times as
it cooled. In the smoker was a calibrated Taylor oven thermometer hanging from 
a rack near the sensor to be measured. The measurements were used to get Steinhart-Hart
coefficients for the thermistor, then plugged back into the program and the process 
repeated to get another set of points. See my 
[ConOp post for details](https://codingstick.com/projects/proj-sch/)
 on the process.

As I refined this program it seemed capable of doing the whole job. It is patterned as 
a so-called superloop. I had already designed another approach based on a 
[finite state machine](https://github.com/mwduffy/ino-SmokerCntlr/) 
but decided to go with this one.

The program has several distinct but related tasks implemented using a
superloop pattern. The Arduino loop() tests elapsed time values to determine if
a task is to be active. If not it simply returns. This works because these tasks are
ordered by increasing time intervals. There are other ways to organize a superloop but 
this works well enough.

The tasks:
- Read analog pins for thermistors and setpoint potentiometer
- Update LCD display
- Modulate heating element based on setpoint and checkpoint adjustment
- Checkpoint to evaluate adjustment for rate of temperature change

The checkpoint idea was added after the decision to extend ThermoSetter from its
calibration goals to become a controller. To deal with the tendency to overshoot the
target, the target became a band of values rather than a single point. A periodic 
assessment of the slope of the temperature change is made and the width of the target 
band is adjusted to minimize overshoot.


