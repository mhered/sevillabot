# NEMA17 Stepper motor

12V 200 steps per revolution NEMA17 motor without datasheet (MMPP004 @ Electronica Embajadores)



Source DroneBot Workshop: https://www.youtube.com/watch?v=0qwrnUeSpYQ

## Unipolar vs bipolar

| Unipolar                            | Bipolar             |
| ----------------------------------- | ------------------- |
| 4 wire                              | 5 or 6 wires        |
| needs voltage reversal              | no voltage reversal |
| Higher torque                       | Lower Torque        |
| Lower Max Speed                     | Higher Max Speed    |
| Requires a more Advanced Controller | Simpler Controller  |



## Selecting a Stepper AKA stepper specs

- Step Angle: in degrees per step or steps per revolution
- Voltage rating
- Current rating
- Coil Resistance
- Inductance: relates to max speed, the higher the slower the motor
- Holding Torque:  max torque when energized
- Detent or Residual Torque: torque when not energized
- Shaft style: round, D, geared, Lead-Screw

## Wiring of a NEMA stepper without diagram

1. Identify the matching pairs checking for continuity with the multimeter
2. Determine the polarity of the two pairs by trial and error using the sample code from `Stepper.h` that can be found in Examples, i.e. until clockwise/counterclockwise work fine in `stepper_oneRevolution` and all steps are in the same direction in `stepper_oneStepAtATime` 