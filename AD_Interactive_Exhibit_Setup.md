# Digitalisation and AI for Anaerobic Digestion (AD)
## Interactive Museum Exhibit: The "Reactor Control Challenge"

### 1. Exhibit Concept & Narrative
The purpose of this exhibit is to tangibly demonstrate the core value of your Supergen Bioenergy Hub project: transitioning the Anaerobic Digestion (AD) sector from **fragmented, manual, spreadsheet-driven operations** to **real-time, data-driven digitalisation using AI and predictive control**.

Visitors will act as "Plant Operators." They will hold a physical box containing an IMU (representing reactor controls) and attempt to keep the biological parameters of an AD reactor stable. 
* **The Problem:** Biological systems have high inertia and delayed feedback. Manual control almost always leads to a reactor crash.
* **The Solution:** By switching to "AI Mode," predictive models anticipate system delays, smooth out noisy inputs, and easily guide the reactor to optimal health.

---

### 2. Parameter Mapping: IMU to AD Biology
A 6-axis IMU can track multiple dimensions. We will map the physical tilting of the box to critical AD operational parameters.

* **Roll (X-axis Tilt) ➔ pH Level:**
  * **Goal:** Maintain between 6.8 and 7.2 (Optimal for methanogenesis).
  * **Mechanic:** Tilting left drops the pH (acidification/souring from too much fast-degrading organics). Tilting right raises the pH (alkalinity buildup). 
* **Pitch (Y-axis Tilt) ➔ Temperature:**
  * **Goal:** Maintain between 35°C and 40°C (Mesophilic conditions).
  * **Mechanic:** Tilting forward increases heating input; tilting backward decreases it.
* **Yaw/Twist (Z-axis) or Accelerometer ➔ Organic Loading Rate (Feed Volume):**
  * **Goal:** Keep the feed steady. 
  * **Mechanic:** Sudden shakes or twists simulate a "shock load" of feedstock, destabilizing the Temp and pH briefly.

---

### 3. Hardware Requirements & Build Setup

**Bill of Materials:**
* 1x Plastic enclosure box (acts as the interactive "Reactor Wand").
* 1x Power bank (fits inside the box).
* 2x Seeed Studio NRF boards (with built-in IMUs).
* 1x Physical Toggle Switch (SPST or SPDT).
* 1x PC/Laptop for the visual dashboard.

**The "Reactor Wand" Build (Inside the Box):**
1. **Power:** Connect one Seeed Studio NRF board to the power bank via USB.
2. **The Switch:** Mount your physical toggle switch to the outside of the plastic box. Wire the switch internally to a digital GPIO pin on the Seeed Studio board and to GND. Enable the internal pull-up resistor in your code.
3. **Mounting:** Firmly secure the power bank, NRF board, and loose wires using double-sided tape or hot glue so nothing moves when the box is tilted.
4. **Enclosure:** Close the box securely.

---

### 4. Software Architecture & Communication Flow

#### Step A: Transmitter (The Box)
Write a C++/Arduino sketch for the Seeed Studio board inside the box.
* **Tasks:**
  1. Read Pitch, Roll, and Yaw from the IMU.
  2. Read the state of the physical toggle switch (`0` for Manual, `1` for AI).
  3. Send this data continuously via BLE/Radio to the receiver dongle.
* **Data Format:** A simple comma-separated string ending in a newline:
  `Pitch,Roll,Yaw,SwitchState
`
  *(Example: `37.5,6.9,12,1
`)*

#### Step B: Receiver (The Bridge)
* Plug the second Seeed Studio NRF board (or a Nice!Nano) into the PC via USB.
* Write a basic script for it to act as a **Serial Bridge**. It receives the wireless packet and uses `Serial.println()` to print that exact comma-separated string to the PC's COM port.

#### Step C: The PC Dashboard (Python Game Engine)
Use **Python** with libraries like `pyserial` (to read data) and `Pygame` or `Streamlit/Dash` (for the UI).

**The UI Layout:**
* **Main Gauges:** Two large, clear gauges for **Temperature** and **pH**. Each has a green "Safe Zone" and red "Danger Zones".
* **Status Indicator:** A large text box showing the current mode: 
  * 🔴 **MANUAL MODE: Direct Control (High Latency)**
  * 🔵 **AI PREDICTIVE MODE: Assisted Control (Optimised)**
* **Warning System:** Flashing alerts if parameters crash (e.g., "SYSTEM SOURING - BACTERIA DYING").

---

### 5. The Core Logic: Manual vs. AI Mode

This is where you simulate the value of your research.

#### Mode 1: Manual Mode ("The Lab / Spreadsheet Era")
* **Trigger:** The physical switch on the box is flipped to `0`.
* **The Logic (Delayed & Noisy):** * Add a **dead-time delay buffer** in your Python code (e.g., a FIFO queue of 2 to 3 seconds). When the user tilts the box, the UI gauges do not respond immediately. 
  * Add a slight "momentum" or drift.
* **The User Experience:** The visitor tilts the box, nothing happens, so they tilt it more. Suddenly, the temperature shoots up to 45°C. They panic, tilt back, and drop the pH to 5.0. The reactor crashes. It feels frustrating, proving humans cannot manage complex, delayed biological systems manually.

#### Mode 2: AI Predictive Mode ("Digitalisation Era")
* **Trigger:** The physical switch on the box is flipped to `1`.
* **The Logic (Smooth & Assisted):**
  * **Remove the delay.** Your Python script intercepts the raw IMU data and passes it through a software filter (representing your hybrid AI/MTGP models).
  * Implement a simple proportional controller (PID) in the code. It maps their hand movements smoothly. If they tilt too far into the danger zone, the "AI algorithm" caps the input, preventing a crash.
* **The User Experience:** The gauges lock smoothly into the Green Zones. The data on screen looks clean, responsive, and safe. The visitor instantly feels the power of AI-assisted digitalisation.

---

### 6. Fallback Alternative: Software Controls

Hardware switches can sometimes fail at museum exhibits, or some users may prefer a purely digital interaction. Ensure your Python script includes a fallback:

* **On-Screen Mode Toggle:** Add a clickable button on the PC UI that says `[Toggle AI Assistant]`. If the user clicks this with a mouse, it overrides the physical switch state coming from the box.
* **On-Screen Sliders (Mouse Control):** Include a hidden debug menu or an alternative layout where users can use their mouse to drag sliders for Temp/pH instead of holding the box. This ensures the exhibit stays functional even if the box battery dies.

---

### 7. Next Steps for Implementation
1. **Flash the Receiver:** Get the PC receiving raw strings via `pyserial`.
2. **Build the Box:** Tape the components together, wire the physical switch, and verify the `Pitch,Roll,Yaw,SwitchState` string changes when you flip the switch.
3. **Write the Python UI:** Start with simple text outputs in the console before building the graphical gauges. Focus heavily on perfecting the "frustrating delay" in Manual Mode to make the AI Mode feel magical in contrast.
