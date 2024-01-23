# Arduino Speedometer 🚵
Fully functonal and optimized arduino speedometer.

## Simulation 📲

The project was entirely simulated on Wokwi as I didn't have a physical Arduino for testing.

The system can be easily replicated in real life, all that is needed is a sensor capable of scanning with a duality of states (HIGH-LOW) for one revolution of the wheel.

## Demo 🛠️

You can try your self the program [here](https://wokwi.com/projects/370181612744975361)

## Optimization 📈

A modular system has been devised to function exclusively through interrupts, devoid of reliance on standard Arduino library functions. The implementation involved crafting functions in C and assembly. Pin state manipulation was achieved through bit shifting, enabling the setting of pins to high or low. The system utilized timer interrupts for precise timing and external interrupts for sensor-related tasks. This approach required a departure from the conventional use of Arduino functions, necessitating a low-level coding strategy.

## Preview 👀

Overview of the project

![Preview](overview_and_wiring.png)

Functioning

![Functioning](functioning.png)

## Flow Chart

![FlowChart](overall_flow_chart.jpg)

## State Chart

![StateChart](statechart.jpg)

## Contact Me 📬

If you want more info about the project please contact me on:

email: francescocalicchio@hotmail.com


