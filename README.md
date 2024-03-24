[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/2TmiRqwI)
# final-project-skeleton

    * Team Name: Sid's Kids
    * Team Members: Rohan Panday and Owen Ledger
    * Github Repository URL: <https://github.com/ese3500/final-project-sid-s-kids>
    * Github Pages Website URL: [for final submission]
    * Description of hardware: (embedded hardware, laptop, etc): MacOS Sonoma, Macbook Pro with M1 Chip

## Final Project Proposal

### 1. Abstract

Our project is a rectifier that takes in an AC input (either 120 VAC from the wall stepped down to 12 VAC or a waveform generator) and use a controlled 4 silicon-controlled rectifier (SCR) rectifier to perform rectification and produce a DC output. The ATMega acts as the controller for the SCRs and performs the firing angle calculations using a PID controller to vary the firing angle as needed based on measured input and output voltages, and generate the actual firing angle pulse. The load we will drive using this controller is a DC motor.

### 2. Motivation

Portable DC power sources have a multitude of uses; whether it is intrumentation, charging batteries, or most portable/consumer electronics, DC power is needed. As Electrical Engineers, we routinely use DC Power Supplies for our work. However, the most widely available and portable power sources tend to be AC power sources (that is what comes out of the wall after all). Furthermore, since we are aiming to drive a DC motor (where we can vary the output voltage to drive the motor faster), we are essentially creating a mini DC motor controller as well, which has myriad applications.

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

This project requires two high-precision differential voltage measurement sensors or integrated circuits (ICs), one placed at the input and the other at the output. These sensors must be capable of:

- Measuring the amplitude of the AC voltage at the input to determine the real-time voltage level, which is crucial for the PID loop to calculate the necessary adjustments for the firing angles of the SCRs.
- Measuring the frequency of the AC voltage. While the system may assume a standard frequency of 60 Hz, the capability to measure the frequency provides flexibility for operation in various power environments and enhances the adaptability of the system.
- Providing accurate DC voltage measurement at the output to facilitate the PID controller in determining the error between the desired and the actual output voltages.
- The measurements obtained from these sensors should be communicated back to the ATMega microcontroller, either through analog-to-digital conversion (ADC) inputs or a digital communication protocol.

The microcontroller, an ATMega 328 PB should be able to:

- Executing the PID control loop algorithm, requiring rapid computation to adjust the firing angles of the SCRs based on the measured input and output voltages.
- Interfacing with the voltage measurement sensors, reading and processing the sensor data to make real-time decisions.
- Managing user inputs and displaying relevant system parameters on an output interface.
- Sending gate pulses in a timely and rapid manner

The hardware will use an LCD display panel to show critical system parameters, such as:

- Input voltage amplitude and frequency.
- Current output voltage.
- Desired output voltage set by the user.

The display should allow users to interactively set the desired output voltage within a predetermined range that considers the limitations imposed by the input voltage and the firing angle possibilities, possibly using a potentiometer to set the desired output voltage.

Safety components such as fuses and diodes will be incorporated to protect both the hardware and the user from electrical hazards. These components are crucial, especially considering the potential high-voltage nature of the system's operation (at least from an embedded systems perspective).

### 6. MVP Demo

What do you expect to accomplish by the first milestone?

### 7. Final Demo

What do you expect to achieve by the final demonstration or after milestone 1?

### 8. Methodology

What is your approach to the problem?

### 9. Components

What major components do you need and why?

### 10. Evaluation

What is your metric for evaluating how well your product/solution solves the problem? Think critically on this section. Having a boolean metric such as “it works” is not very useful. This is akin to making a speaker and if it emits sound, albeit however terrible and ear wrenching, declare this a success.
It is recommended that your project be something that you can take pride in. Oftentimes in interviews, you will be asked to talk about projects you have worked on.

### 11. Timeline

This section is to help guide your progress over the next few weeks. Feel free to adjust and edit the table below to something that would be useful to you. Really think about what you want to accomplish by the first milestone.

| **Week**            | **Task** | **Assigned To**    |
|----------           |--------- |------------------- |
| Week 1: 3/24 - 3/31 |          |                    |
| Week 2: 4/1 - 4/7   |          |                    |
| Week 3: 4/8 - 4/14  |          |                    |
| Week 4: 4/15 - 4/21 |          |                    |
| Week 5: 4/22 - 4/26 |          |                    |

### 12. Proposal Presentation

Add your slides to the Final Project Proposal slide deck in the Google Drive.

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

#### 3.2 Hardware Requirements Specification (HRS) Results

Based on your quantified system performance, comment on how you achieved or fell short of your expected hardware requirements. You should be quantifying this, using measurement tools to collect data.

### 4. Conclusion

Reflect on your project. Some questions to consider: What did you learn from it? What went well? What accomplishments are you proud of? What did you learn/gain from this experience? Did you have to change your approach? What could have been done differently? Did you encounter obstacles that you didn’t anticipate? What could be a next step for this project?

## References

Fill in your references here as you work on your proposal and final submission. Describe any libraries used here.

## Github Repo Submission Resources

You can remove this section if you don't need these references.

* [ESE5160 Example Repo Submission](https://github.com/ese5160/example-repository-submission)
* [Markdown Guide: Basic Syntax](https://www.markdownguide.org/basic-syntax/)
* [Adobe free video to gif converter](https://www.adobe.com/express/feature/video/convert/video-to-gif)
* [Curated list of example READMEs](https://github.com/matiassingers/awesome-readme)
* [VS Code](https://code.visualstudio.com/) is heavily recommended to develop code and handle Git commits
  * Code formatting and extension recommendation files come with this repository.
  * Ctrl+Shift+V will render the README.md (maybe not the images though)
