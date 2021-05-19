# Closed-Loop-BLDC-motor-control-using-dsPic30F2010.

The dsPIC30F2010 is a 28-pin 16-bit MCU specifically
designed for embedded motor control applications. AC
Induction Motors (ACIM), Brushless DC (BLDC) and
DC are some typical motor types for which the
dsPIC30F2010 has been specifically designed. Some
of the key features on the dsPIC30F2010 are:
• 6 independent or 3 complementary pairs of
dedicated Motor Control PWM outputs.
• 6 input, 500Ksps ADC with up to 4 simultaneous
sampling capability.
• Multiple serial communications: UART, I2C™ and
SPI
• Small package: 6 x 6 mm QFN for embedded
control applications
• DSP engine for fast response in control loops.

BLDC motors are basically inside-out DC motors. In a
DC motor the stator is a permanent magnet. The rotor
has the windings, which are excited with a current. The
current in the rotor is reversed to create a rotating or
moving electric field by means of a split commutator
and brushes. On the other hand, in a BLDC motor the
windings are on the stator and the rotor is a permanent
magnet. Hence the term inside-out DC motor.
To make the rotor turn, there must be a rotating electric
field. Typically a three-phase BLDC motor has three
stator phases that are excited two at a time to create a
rotating electric field. This method is fairly easy to
implement, but to prevent the permanent magnet rotor
from getting locked with the stator, the excitation on the
stator must be sequenced in a specific manner while
knowing the exact position of the rotor magnets.
Position information can be gotten by either a shaft encoder or, more often, by Hall effect sensors that
detect the rotor magnet position. For a typical threephase,
sensored BLDC motor there are six distinct
regions or sectors in which two specific windings are
excited.

![image](https://user-images.githubusercontent.com/80693305/118834812-c39dd880-b8c2-11eb-888e-26f71f4b514e.png)


By reading the Hall effect sensors, a 3-bit code can be
obtained with values ranging from 1 to 6. Each code
value represents a sector on which the rotor is
presently located. Each code value, therefore, gives us
information on which windings need to be excited. Thus
a simple lookup table can be used by the program to
determine which two specific windings to excite and,
thus, turn the rotor.
Note that state ‘0’ and ‘7’ are invalid states for Hall
effect sensors. Software should check for these values
and appropriately disable the PWM. Taking this technique a step further, the Hall effect
sensors can be connected to dsPIC30F2010 inputs
that detect a change (Change Notification (CN) inputs).
An input change on these pins generates an interrupt.
In the CN Interrupt Service Routine (ISR) the user
application program reads the Hall effect sensor value
and uses it to generate an offset in the lookup table for
driving the windings of the BLDC motor.

![image](https://user-images.githubusercontent.com/80693305/118835334-2d1de700-b8c3-11eb-8fe9-56308b34b09b.png).

In the closed-loop control version of the firmware, the
main difference is that the pot is used for setting the
demand. The control loop provides Proportional and
Integral (PI) control of the speed. To measure the
actual speed, TMR3 is used as a timer to gate a complete
electrical cycle. Since we are using a 10-pole
motor, five electrical cycles result in one mechanical
cycle. If T seconds is the time for one electrical cycle
then the speed S = 60/(P/2*T) rpm, where P is the
number of poles of the motor.

![image](https://user-images.githubusercontent.com/80693305/118835585-6d7d6500-b8c3-11eb-80fd-2f04bdd13341.png)




