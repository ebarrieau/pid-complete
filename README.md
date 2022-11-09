# node-red-contrib-pid-complete# node-red-contrib-pid-complete

A node-red node to performe PID Calculations

# Getting started

## pid-complete node

### Input

1. msg.payload - must contain the input process variable (PV) unless a command is send.
2. msg.cmd - can contain `reset`, `disable`, `auto`, or `manual`.

1. msg.payload - contains the output setpoint if in Auto or Manual.
2. msg.proportional - contains the output of the last proportional calculation.
3. msg.integral - contains the output of the last integral calculation.
4. msg.derivative - contains the output of the last derivative calculation.

### Configuration

| Setting                  | Description                                                                  |
| ------------------------ | -----------------------------------------------------------------------------|
| `Name`                   | What ever you name the node                                                  |
| `Calculation Interval`   | Time between calculations. Set longer for slow processes.                    |
| `Min Output`            | Sets the lower range of the output calculation.                              |
| `Max Output`             | Sets the upper range of the output calculation.                              |
| `Kp`                     | Proportional Gain.                                                           |
| `Ki`                     | Integral Gain.                                                               |
| `Kd`                     | Derivative Gain.                                                             |
| `SP`                     | Temperature Setpoint                                                         |   |
| ``'Bump Percent`           | Output step as percent of setpoint for autotuning - In Progress              \


# Contributing

1. Fork this repo
2. Write a red unit test for your change
3. Implement the code and make the test green
4. Refactor your code to make it nice
5. Make a pull request

I will probably approve it ;)

# Credits

Special thanks to Martin Lundberg https://github.com/m-lundberg/simple-pid and Brett Beauregard https://github.com/br3ttb/Arduino-PID-Library who's work this library is based on.
