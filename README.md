[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/2TmiRqwI)
# final-project-skeleton

    * Team Name: Sid's Kids
    * Team Members: Rohan Panday and Owen Ledger
    * Github Repository URL: <https://github.com/ese3500/final-project-sid-s-kids>
    * Github Pages Website URL: [for final submission]
    * Description of hardware: (embedded hardware, laptop, etc): MacOS Sonoma, Macbook Pro with M1 Chip

## Final Project Proposal

### 1. Abstract

In a few sentences, describe your final project. This abstract will be used as the description in the evaluation survey forms.

Our idea is to create a rectifier that will take in an AC input (likely stepped down from the 120 VAC from the wall for this purpose) and use a controlled 4 SCR rectifier to perform rectification and produce a DC output. The ATMega will act as the controller for the SCRs and will perform the firing angle calculations, use a PID controller to vary the firing angle as needed based on measured input and output voltages, and generate the actual firing angle pulse.

### 2. Motivation

What is the problem that you are trying to solve? Why is this project interesting? What is the intended purpose?

Portable DC power sources have a multitude of uses; whether it is intrumentation, charging batteries, or most portable/consumer electronics, DC power is needed. As Electrical Engineers, we routinely use DC Power Supplies for our work. However, the most widely available and portable power sources tend to be AC power sources (that is what comes out of the wall after all).

### 3. Goals

These are to help guide and direct your progress.

### 4. Software Requirements Specification (SRS)

Formulate key software requirements here.

Since we are running PID on discrete data that is running at relatively high frequencies, we need to be able to compute the PID equation very quickly (which should be fine) and solve for the firing angle quickly (which may require a lookup table of inverse cosine). Once a firing angle has been calculated, we will need to generate a pulse at that firing angle (which will require us to know the input frequency). This firing angle will change cycle to cycle based on the controller during transition regions, so we will need to be able to time and modulate the OCRA values well to control the SCRs properly. The output voltage will be measured and used as an input to the PID controller.

### 5. Hardware Requirements Specification (HRS)

Formulate key hardware requirements here.

The sensors we would need would be two differential voltage measurement sensors/ICs (one at the input and one at the input). At the input, we would measure the amplitude and frequency of the AC voltage (if frequency is too difficult we can assume 60 Hz, but if we can measure amplitude we should be able to do frequency). Then, these measurements (which would either come back to the ATMega through the ADC or through a communications protocol), would be used. For the PID loop, the target is the user's desired output, and the parameter to measure in order to calculate error is the measured output voltage. We have a transfer function that describes how the input voltage, firing angle, and output voltage are related, so we can create our PID equation.

The output will be an LCD display that shows the input and output voltage characteristics, and allows the user to change the desired output voltage (we give a range given the input voltage and firing angle possibilities).

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
