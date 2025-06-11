# The Beginning

At the beginning of the year, January 28, this project was created. The goal back then is the same as the current goal, to create a MIDI controller to be used for my church's worship team. Progress was slow, and by February 20, the project was postponed and abandoned. The reason for that was because I was naive enough to think that I could create my own MIDI commnication protocol. There were successes with the CDC class, but because of this [STM32 document](https://www.st.com/content/ccc/resource/technical/document/user_manual/cf/38/e5/b5/dd/1d/4c/09/DM00108129.pdf/files/DM00108129.pdf/jcr:content/translations/en.DM00108129.pdf) I found that said that I needed the Audio USB class to be able to implement the MIDI communicatio protocol, I hit a road block that no amount of online help could get me through.

The project was revived at the very beginning of June, when I stumbled across this [git repo](https://github.com/Hypnotriod/midi-box-stm32) by Hypnotroid that provided a simple MIDI protocol class. This class utilized the HID class instead of the Audio class that STM32 had recommended, and after configuring my project just like how the tutorial said, I was able to *finally* send MIDI messages using a custom USB male port to female pins cable. 

Progress on the prototype quickened and a few days later, the push buttons were functional and could send the proper MIDI messages. I also tested the controller on MainStage, the program I use for the worship team, and after a little bit of trial and error, the program was able to consistently connect to the device and respond to the messages being sent.

The potentiometer feature was added another few days later, with some issues. The ADC pin on the MCU was too sensitive, and would constantly change its value even when the potentiometer wasn't moved. I still have to test that and remove as much error from it before moving on to the final design. 

On the last day of the week, I was able to move to the second phase of prototyping, switching over to the STM32 MCU that will be used for the final project. The process was relatively simple; it was just a matter of rewiring the breadboard to the new MCU, and setting up the code and project on the IDE.

This week brought about quick progress, but the project is likely to slow down as I start crossing uncharted territory, namely PCB designing and soldering. The plan for next week is to work on the PCB and hopefully send the design to be printed. 
