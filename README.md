# Rohan and Owen: SCR-Based Rectifier

    * Team Name: Sid's Kids
    * Team Members: Rohan Panday and Owen Ledger
    * Github Repository URL: <https://github.com/ese3500/final-project-sid-s-kids>
    * Github Pages Website URL: https://ese3500.github.io/final-project-sid-s-kids/
    * Description of hardware: (embedded hardware, laptop, etc): MacOS Sonoma, Macbook Pro with M1 Chip


## Final Project Report

Don't forget to make the GitHub pages public website!
If you’ve never made a Github pages website before, you can follow this webpage (though, substitute your final project repository for the Github username one in the quickstart guide):  <https://docs.github.com/en/pages/quickstart>

### 1. Video

[Insert final project video here]

### 2. Images

[Insert final project images here]

### 3. Results

What were your results? Namely, what was the final solution/design to your problem?

#### 3.1 Software Requirements Specification (SRS) Results

Based on your quantified system performance, comment on how you achieved or fell short of your expected software requirements. You should be quantifying this, using measurement tools to collect data.

#### 3.1.1 Overview

The software is an integral part of a rectifier system and manages the conversion process from AC to DC by controlling the firing angle of SCRs using a PID algorithm on an ATMega 328 PB microcontroller.

#### 3.1.2 Users

The software is designed for electrical engineers and technicians responsible for maintaining stable DC power supplies for applications such as DC motors.

#### 3.1.3 Definitions, Abbreviations

- **SCR**: Silicon Controlled Rectifier
- **PID**: Proportional-Integral-Derivative
- **ADC**: Analog to Digital Converter
- **DC**: Direct Current
- **AC**: Alternating Current
- **Cuk Converter**: A DC to DC converter that can both buck and boost voltage
- **Buck**: To lower DC voltage, normally with a switching converter
- **Boost**: To increase DC voltage, normally with a switching converter

#### 3.1.4 Functionality

- **SRS 01**: The software shall implement a PID control algorithm to regulate the firing angle of SCRs.
- **SRS 02**: The software shall continuously monitor input and output voltage levels through ADCs.
- **SRS 03**: The software shall provide real-time response to voltage fluctuations and load conditions.
- **SRS 04**: The microcontroller shall compute the PID equation efficiently, possibly utilizing lookup tables for inverse trigonometric functions.
- **SRS 05**: The software shall generate precise firing pulses synchronized with the AC frequency.
- **SRS 06**: The software should allow dynamic adjustment of firing angles for varying operating conditions.
- **SRS 07**: The software must provide a mechanism for setting and tuning initial PID parameters.
- **SRS 08**: The software must include robust error handling capabilities.

#### 3.2 Hardware Requirements Specification (HRS) Results

Based on your quantified system performance, comment on how you achieved or fell short of your expected hardware requirements. You should be quantifying this, using measurement tools to collect data.

#### 3.2.1 Overview

The hardware facilitates voltage measurement and PID control loop execution for a rectifier system, utilizing an ATMega 328 PB microcontroller as the core component.

#### 3.2.2 Definitions, Abbreviations

- **PID**: Proportional-Integral-Derivative
- **SCR**: Silicon Controlled Rectifier
- **ADC**: Analog to Digital Converter
- **LCD**: Liquid Crystal Display

#### 3.2.3 Functionality

- **HRS 01**: An op-amp circuit shall be used to scale AC input voltages into the ADC's range.
- **HRS 02**: The ATMega 328 PB microcontroller shall execute the PID control loop algorithm for adjusting SCR firing angles.
- **HRS 03**: The microcontroller shall interface with voltage measurement sensors for real-time decision-making.
- **HRS 04**: The microcontroller shall manage user inputs and display system parameters on an LCD panel.
- **HRS 05**: The hardware shall send gate pulses to SCRs in a timely and rapid manner.
- **HRS 06**: The system shall use an LCD display panel for showing system parameters and user interaction.
- **HRS 07**: Safety components such as fuses and diodes shall be incorporated to protect against electrical hazards.

### 4. Conclusion

Reflect on your project. Some questions to consider: What did you learn from it? What went well? What accomplishments are you proud of? What did you learn/gain from this experience? Did you have to change your approach? What could have been done differently? Did you encounter obstacles that you didn’t anticipate? What could be a next step for this project?

## References

Fill in your references here as you work on your proposal and final submission. Describe any libraries used here.

-----

[1] L. Gu, Class Lecture, Topic: "Thyristors” ESE 5800, School of Engineering and Applied Sciences, University of Pennsylvania, Philadelphia, Pennsylvania, Feb., 7, 2024.

[2] J. G. Kassakian, D. J. Perreault, G. C. Verghese, and M. F. Schlecht, Principles of Power Electronics, 2nd ed. Cambridge: Cambridge University Press, 2023.

[3] “Control Tutorials for MATLAB and Simulink - Introduction: PID Controller Design,” ctms.engin.umich.edu. https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID

[4] L. Gu, Class Lecture, Topic: "Switching Converters” ESE 5800, School of Engineering and Applied Sciences, University of Pennsylvania, Philadelphia, Pennsylvania, Feb., 7, 2024.

## Final Project Proposal

### 1. Abstract

Our project is a rectifier that takes in an AC input (either 120 VAC from the wall stepped down to 12 VAC or a waveform generator) and use a controlled 4 silicon-controlled rectifier (SCR) rectifier to perform rectification and produce a DC output. The ATMega acts as the controller for the SCRs and performs the firing angle calculations using a PID controller to vary the firing angle as needed based on measured input and output voltages, and generate the actual firing angle pulse. The load we will drive using this controller is a DC motor.

### 2. Motivation

Portable DC power sources have a multitude of uses; whether it is instrumentation, charging batteries, or most portable/consumer electronics, DC power is needed. As Electrical Engineers, we routinely use DC Power Supplies for our work. However, the most widely available and portable power sources tend to be AC power sources (that is what comes out of the wall after all). Furthermore, since we are aiming to drive a DC motor (where we can vary the output voltage to drive the motor faster), we are essentially creating a mini DC motor controller as well, which has myriad applications.

### 3. Goals

- Create a rectifier circuit using SCRs on the prototype (this can be tested using the waveform generators in the lab)
- Design the PID controller for this
- Connect the voltage measurement sensors to the rectifier
- Implement the PID controller in software
- Implement the firing angle code on the ATMega
- Combine the firing angle code with the PID control loop

### 4. Software Requirements Specification (SRS)

The software for this project is designed to run on an ATMega 328 PB microcontroller, which acts as the controller for the rectifier. The primary function of the software is to implement a PID control algorithm that regulates the firing angle of SCRs to convert alternating current (AC) input into direct current (DC) output. The software will continuously monitor the input and output voltage levels through ADCs or differential voltage sensors and adjust the firing angle of the SCRs accordingly to maintain a stable DC output voltage, suitable for applications such as driving DC motors.

The software is required to perform high-frequency calculations to ensure real-time responses to fluctuations in input voltage and load conditions. It should efficiently compute the PID equation, enabling the microcontroller to solve for the firing angle quickly, potentially using lookup tables for inverse trigonometric functions to expedite calculations. It may need to compute some complex numbers to apply the transfer function of the output filter, though we could get around this by working with the PID controller in the laplace domain and then doing all conversions at the end. The software must generate precise firing pulses, taking into account the input AC frequency, to control the SCRs accurately. This control logic must allow for dynamic adjustment of the firing angles to cater to different operating conditions, such as changes in the load or input voltage.

Additionally, the software on the backend should be designed to provide mechanisms for setting initial PID parameters and allow for their adjustment/tuning. The software must ensure robust error handling to cope with potential hardware faults or unexpected input signals.

### 5. Hardware Requirements Specification (HRS)

This project requires a way to measure input voltages, in our case using an op-amp circuit. This circuit will be capable of shifting and scaling the input voltages into the ADC range, allowing us to measure the amplitude and frequency of the AC input voltage, and the amplitude of the output voltage.

The microcontroller, an ATMega 328 PB should be able to:

- Executing the PID control loop algorithm, requiring rapid computation to adjust the firing angles of the SCRs based on the measured input and output voltages.
- Interfacing with the voltage measurement sensors, reading and processing the sensor data to make real-time decisions.
- Managing user inputs and displaying relevant system parameters on an output interface.
- Sending gate pulses in a timely and rapid manner
- All these functions will likely require 3 timers (which the ATmega has)

The hardware will use an LCD display panel to show critical system parameters, such as:

- Input voltage amplitude and frequency.
- Current output voltage.
- Desired output voltage set by the user.

The display should allow users to interactively set the desired output voltage within a predetermined range that considers the limitations imposed by the input voltage and the firing angle possibilities, possibly using a potentiometer to set the desired output voltage.

Safety components such as fuses and diodes will be incorporated to protect both the hardware and the user from electrical hazards. These components are crucial, especially considering the potential high-voltage nature of the system's operation (at least from an embedded systems perspective).

### 6. MVP Demo

A silicon controlled rectifier circuit with at least a constant firing angle. Ideally a voltage output measurement would be included as well.

The demonstration will consist of the following: attaching an AC power source to the device either from a transformer or waveform generator and measuring the output with the device and an osciliscope in order to examine ripple and accuracy to most likely a 1/2 voltage scaling. A constant firing angle alpha will be employed at this point which will yield a preset output DC voltage. The goal of this demo is to prove that the SCR devices are working, do not short unintentionally and can handle a DC load.

The basis of the PID controller should be evident in the code, even if it isn't connected to all the sensors.

MVP Slides: <https://docs.google.com/presentation/d/1lvPgcVklSQcYiyNu-Z0bmYHBMlA3lIVZDFER-7pJIpA/edit?usp=sharing>

### 7. Final Demo

A fully closed loop PID controlled SCR voltage device that takes a target DC voltage input in and outputs such voltage.

This will be conducted by connecting the device to a DC E-load, providing it again an arbitrary input waveforem and now setting a DC output voltage with a potentiometer on the housing. The DC output voltage will be readout on an LCD as well as the target voltage and characteristics of the input waveforem to be compared with what is actually being generated and taken in by the device.

If we are able to get this part fully working, we will attempt to drive a DC motor using our controller.

### 8. Methodology

First off, we will figure out what we need, like inputs/outputs, how fast and accurate it needs to be, making sure it's safe, and deciding how we want it to look and feel for users. Then, we will work to acquire parts like SCRs, accurate sensors for voltage and current, the ATMEGA328PB Xplained Board, and some other components like an LCD screen and a potentiometer. Then, we will wire the circuit and test the rectifier circuit without the ATMega.

At the same time, we will simultaneously develop the PID control algorithm for the ATMega 328 PB microcontroller. We will be setting up how we'll retrieve data from sensors and tune the controller to keep our firing angles on point. We will use tools like MatLab to test and refine our PID control.

We're also planning to make the design user-friendly and reliable, with clear error messages and nice features on the LCD so everyone can use it. After all that, it's testing time! We'll check each part by itself and then see how they all play together, fixing any problems along the way.

Next up, we'll bring all the software and hardware together, tuning all the settings like PID coefficients, where we want our voltage to be, and the firing angles to get everything running. The last big step is making sure everything we did meets the goals we set, with tests in different conditions to see how it holds up.

And we can't forget about wrapping everything up with some solid documentation, including diagrams of how the system's put together, all the circuit details, the code, how we got it all calibrated, and every test we ran.

### 9. Components

Primary components:
* ATMEGA328PB Xplained Board
* 4 High Power Silicon Controlled Rectifier Devices
* Power inductor
* High voltage capacitor
* Voltage sensing equipment (now, it is 2 op amp circuits)
* LCD screen
* Potentiometer
* Diodes for input protection

### 10. Evaluation

There are two primary metrics by which we can evaluate our project: a boolean one, does it take in an arbitrary AC waveform within our bounds in and output a rectified regulated voltage or current based on the input given by the user. The more quantifiable way to approach this would be how fast can the system output the desired quality within some time frame. The boundaries and thresholds for this qualification will be something better understood in development, but an example would be, "the device can regulate to +/- 0.05V from target within 5 seconds" or something similar. A target, for now, before any simulation/development has begun, will be for the rectifier to be able to regulate to within 0.25V within 5 seconds.

### 11. Timeline

| **Week**            | **Task** | **Assigned To**    |
|----------           |--------- |------------------- |
| Week 1: 3/24 - 3/31 | Select and order SCRs, protection diodes, voltage sensing methods and screen. |Owen
||Begin simulation and determine LC values|          Rohan            |
| Week 2: 4/1 - 4/7   | Write gate pulse, frequency analysis and LCD code.|     Owen
|| Simulate PID control and tune for inductor and capacitor amd write code for it.              | Rohan
| Week 3: 4/8 - 4/14  | Assemble circuitry and begin integration.|           Both         |
| Week 4: 4/15 - 4/21 |  Debugging.  Housing for the device.       |          Both          |
| Week 5: 4/22 - 4/26 |   Debugging. Polish and final testing       |         Both           |

### 12. Proposal Presentation

Add your slides to the Final Project Proposal slide deck in the Google Drive.