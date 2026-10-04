# Corrected and Annotated Note 1: Foundations of Antenna Theory and Design

This note serves as a comprehensive, corrected, and scientifically grounded reference for the fundamental principles of electromagnetics, transmission lines, and antenna parameters, specifically tailored toward the design of a **16-element microstrip patch antenna array**.

---

## 1. Antenna Definition and the Nature of Guided vs. Free-Space Waves

### **What is an Antenna?**
An antenna is fundamentally a **transition device** (or transducer) that interfaces a guided electromagnetic wave on a transmission line (or waveguide) with an unguided wave propagating through free space (or another medium) [546, 748]. 

### **What is Traveling Through the Transmission Line?**
In your note, you asked whether the signal traveling through the transmission line is a voltage wave, an electromagnetic (EM) wave consisting of electric ($\bar{E}$) and magnetic ($\bar{H}$) fields, or an alternating current (AC). 

The correct answer is that **it is all of these simultaneously, viewed from different physical perspectives** [171, 213]:
1. **The Circuit Viewpoint (Voltages and Currents):** In a two-conductor transmission line (such as a coaxial cable or microstrip line), high-frequency alternating currents (AC) flow in the conductors, which in turn establishes a time-varying potential difference (voltage wave) between the conductors [171, 213].
2. **The Field Viewpoint (Electromagnetic Fields):** These time-varying voltages and currents physically manifest in the surrounding dielectric substrate as guided electromagnetic fields [213, 281]. For two-conductor lines, these fields propagate in the **Transverse Electromagnetic (TEM) mode**, meaning both the electric field ($\bar{E}$) and the magnetic field ($\bar{H}$) are entirely transverse (perpendicular) to the direction of wave propagation along the line [213, 220]. 
3. **Equivalency:** The voltage $V(z)$ is uniquely defined by integrating the transverse electric field $\bar{E}$ between the conductors, and the current $I(z)$ is uniquely defined by integrating the magnetic field $\bar{H}$ around the conductor via Ampere’s Law [281, 282]. Thus, the circuit and field descriptions are mathematically equivalent representations of the same physical wave [171, 282].

### **The Meaning of a Transmission Line**
A transmission line is a distributed physical structure whose **physical length ($L$) is comparable to or larger than the electrical wavelength ($\lambda$)** of the high-frequency signal passing through it (typically when $L \ge 0.1\lambda$ or $L \ge \lambda/10$) [61, 351]. 
* At low frequencies (where $\lambda$ is extremely large), the phase of the voltage and current is essentially uniform across the entire device, allowing for the "lumped element" approximations of standard circuit theory [61, 279]. 
* At microwave frequencies (where $\lambda$ is very small), significant phase shifts occur over the physical dimensions of the line. The line must be treated as a **distributed element**, where wave propagation and reflections must be modeled [61, 280].

---

## 2. Field Analysis: Divergence vs. Curl and the Vector Wave

You raised an excellent question regarding why we consider the **divergence** of the electric field at the antennas, but the **curl** of the field in free space. This is explained rigorously by Maxwell's Equations and boundary conditions.

### **Maxwell’s Equations in Phasor Form**
In a linear, isotropic, homogeneous medium with permittivity $\epsilon$ and permeability $\mu$, time-harmonic electromagnetic fields (with an implied $e^{j\omega t}$ time dependence) are governed by [80, 88]:
$$\nabla \times \bar{E} = -j\omega\mu\bar{H} - \bar{M} \quad \text{(Faraday's Law)}$$
$$\nabla \times \bar{H} = j\omega\epsilon\bar{E} + \bar{J} \quad \text{(Ampere's Law)}$$
$$\nabla \cdot \bar{D} = \rho \quad \text{(Gauss's Law for Electricity, where } \bar{D} = \epsilon\bar{E}\text{)}$$
$$\nabla \cdot \bar{B} = 0 \quad \text{(Gauss's Law for Magnetism, where } \bar{B} = \mu\bar{H}\text{)}$$

### **At the Antennas (Divergence and Charges)**
At both the transmitting and receiving antennas, we have physical metal conductors (such as copper patches or feedlines) [1, 29, 253]. 
* **Transmitting Antenna:** When the high-frequency voltage wave arrives at the radiating patch, it drives a time-varying volume charge density ($\rho$) and surface charge density ($\rho_s$) on the metallic patch [71, 91]. According to Gauss's Law ($\nabla \cdot \bar{D} = \rho$), these charges act as the physical sources where electric flux lines originate (diverge outward) [71, 73, 91]. Hence, at the transmitting antenna boundary, we focus on the **divergence** of the fields because the physical charges on the metal are pushing electric field lines out into space [91, 96].
* **Receiving Antenna:** When an incoming wave strikes the receiving patch, the electric field exerts forces on the free electrons in the copper, inducing a surface charge density ($\rho_s$) [91, 96]. Here, Gauss's Law at the boundary ($\hat{n} \cdot \bar{D} = \rho_s$) relates the incoming electric field's termination (divergence) to the induced charges that are converted back into a guided voltage signal [91, 96].

### **In Free Space (Curls and Self-Sustaining Wave Propagation)**
Once the wave detaches from the transmitting antenna and enters free space, there are no longer any physical conductors, free charges, or source currents ($\rho = 0$ and $\bar{J} = 0$) [99, 100]. 
* Because $\rho = 0$, Gauss's Law simplifies to:
  $$\nabla \cdot \bar{E} = 0$$
  This means that in free space, the **divergence of the electric field is zero** [100, 116]! Electric field lines do not start or end in empty space; they must form closed loops.
* Instead, wave propagation in free space is governed entirely by the **coupled curl equations** [100]:
  $$\nabla \times \bar{E} = -j\omega\mu\bar{H}$$
  $$\nabla \times \bar{H} = j\omega\epsilon\bar{E}$$
  A time-varying magnetic field curls to create a time-varying electric field, which in turn curls to recreate the time-varying magnetic field [100]. This continuous, mutually inductive "curling" allows the transverse electromagnetic (TEM) plane wave to propagate through empty space infinitely without needing physical charges to support it [100, 103].

### **What is the "Perpendicular Field" in an Electromagnetic Wave?**
In your note, you asked about the "other, perpendicular field" in an EM wave. 
A propagating plane wave in free space consists **only of two fields**: the electric field ($\bar{E}$) and the magnetic field ($\bar{H}$) [103, 118]. They are mutually perpendicular, and both are perpendicular to the direction of propagation ($\hat{n}$ or $\hat{z}$) [103, 118].
* There is **no third field**. 
* The vector perpendicular to both $\bar{E}$ and $\bar{H}$ represents the **direction of wave propagation** (the wave vector $\bar{k}$) [115].
* The cross-product of the fields defines the **Poynting Vector** ($\bar{S}$):
  $$\bar{S} = \bar{E} \times \bar{H}^* \quad (\text{W/m}^2)$$
  The Poynting Vector represents the **flow of electromagnetic power density** (energy per unit time per unit area) transporting through space in the direction of propagation [130, 550].

---

## 3. Why Radiation Requires Accelerated Charges

You correctly noted that a direct current (DC) cannot radiate and that radiation requires an AC signal (charges in motion). Here is the physical explanation:

1. **Static Charges (Velocity = 0):** Stationary charges produce a static electric field (electrostatics) but no magnetic field. No wave is created.
2. **Constant Velocity Charges (DC Current):** Charges moving at a constant, uniform velocity (DC) produce a static magnetic field (magnetostatics) but no time-varying electric field [73]. No wave is created.
3. **Accelerated Charges (AC Current / Oscillating Velocity):** To launch a propagating wave, we must generate time-varying fields ($\partial \bar{B}/\partial t$ and $\partial \bar{D}/\partial t$), which requires a time-varying current source ($\bar{J}(t)$) [71, 75]. An AC current represents charges that are continuously oscillating back and forth—meaning they are **constantly accelerating and decelerating**. Therefore, electromagnetic radiation is fundamentally a phenomenon of **accelerating charges**.

---

## 4. Input Impedance: Meaningful vs. Meaningless Loss

The input impedance of an antenna is a complex quantity [296, 634]:
$$Z_{in} = R_{in} + jX_{in}$$

The real, resistive part ($R_{in}$) represents power that is drawn from the transmission line and permanently removed from the guided system [296]. We split this resistance into two components [634, 753]:
$$R_{in} = R_{rad} + R_{loss}$$

### **$R_{rad}$ (Radiation Resistance) — The "Meaningful Loss"**
This is the **meaningful "loss"** [634]. It represents the power that is successfully converted from a guided wave on the transmission line into a propagating electromagnetic wave in free space [753, 756]. This power is radiated away to perform the antenna's primary job [753].
* For an edge-fed rectangular microstrip patch antenna, the input resonant resistance at the edge ($y_0 = 0$) is given by [4]:
  $$R_{in}(0) = \frac{1}{2(G_1 \pm G_{12})}$$
  where $G_1$ is the self-conductance of the radiating slot, and $G_{12}$ is the mutual conductance between the two slots [4].
* To match this high edge resistance to a standard $50\ \Omega$ feedline, an **inset feed** at a distance $y_0$ from the patch edge can be used, which reduces the input resistance according to [4]:
  $$R_{in}(y_0) = R_{in}(0) \cos^2\left(\frac{\pi}{L} y_0\right)$$

### **$R_{loss}$ (Loss Resistance) — The "Meaningless Loss"**
This is the **meaningless loss** [634]. It represents power lost as heat due to physical imperfections in the materials [753]. In a microstrip patch antenna, this consists of [82, 259]:
1. **Conduction (Ohmic) Losses ($R_c$):** Due to the finite conductivity of the copper patch and ground plane [259, 260].
2. **Dielectric Losses ($R_d$):** Due to molecular friction and damping inside the substrate (e.g., FR4 or Rogers), quantified by the substrate's dielectric loss tangent ($\tan \delta$) [82, 83, 259].

### **Radiation Efficiency ($\eta_{rad}$)**
The radiation efficiency measures how much of the power accepted by the antenna is radiated rather than lost as heat [560]:
$$\eta_{rad} = \frac{P_{rad}}{P_{in}} = \frac{R_{rad}}{R_{rad} + R_{loss}}$$

### **The Vector Network Analyzer (VNA) "Catch"**
You correctly identified a major "catch" in RF measurement: a standard VNA connected to the antenna terminal measures the total input impedance $Z_{in} = R_{in} + jX_{in}$ [300, 311]. **The VNA cannot tell you how much of the measured $R_{in}$ is $R_{rad}$ and how much is $R_{loss}$** because both look like real power absorption from the feedline's perspective. 
* To measure the true radiation efficiency ($\eta_{rad}$), you must measure the actual radiated far-field power in an anechoic chamber and compare it to the input power accepted by the antenna [554, 773, 775].

---

## 5. Reactance ($X_{in}$) and Antenna Resonance

The imaginary, reactive part of the input impedance ($X_{in}$) represents **stored energy** in the near-field of the antenna [296].
* If $X_{in} > 0$, the antenna stores net magnetic energy ($W_m > W_e$), behaving inductively [296].
* If $X_{in} < 0$, the antenna stores net electric energy ($W_e > W_m$), behaving capacitively [296].

### **Antenna Resonance**
Resonance is defined as the frequency at which the stored magnetic and electric energies are perfectly balanced ($W_m = W_e$), causing the input reactance to vanish [366]:
$$X_{in} = 0$$
At this specific frequency, the input impedance is **purely real** ($Z_{in} = R_{in}$) [366].

### **Why we must minimize $X_{in}$**
An antenna does not stop radiating just because it is non-resonant. However, if $X_{in}$ is large, the antenna impedance $Z_{in}$ will mismatch severely with the transmission line's characteristic impedance $Z_0$ (which is real-valued, usually $50\ \Omega$) [181, 637]. This mismatch causes the incoming wave to reflect back toward the generator, resulting in high **mismatch loss** [181, 691]. Thus, we must design the antenna to operate at resonance ($X_{in} \approx 0$) so we can match $R_{in} \approx Z_0$ and maximize power transfer into the antenna [637, 755].

---

## 6. The Transmission Line Input Impedance Formula

Your note contained a couple of minor mathematical and terminological errors regarding the transmission line impedance equation. 

### **The Correct Impedance Formula**
When looking into a lossless transmission line of characteristic impedance $Z_0$ and physical length $L$ terminated with an antenna impedance $Z_A$, the input impedance $Z_{in}$ is given by [184]:
$$Z_{in} = Z_0 \frac{Z_A + j Z_0 \tan(\beta L)}{Z_0 + j Z_A \tan(\beta L)}$$
* **Correction:** The argument of the tangent function must be the electrical length **$\beta L$**, not just $\beta$. 

### **Terminological Corrections**
* **$\beta$ (Beta):** $\beta$ is the **wavenumber** (or **phase constant**), defined as [100, 104]:
  $$\beta = \frac{2\pi}{\lambda}$$
  It represents the phase shift per unit length along the line (measured in rad/m) [100]. The term "wave index" is incorrect; the correct term is wavenumber.
* **$\tan(\beta L)$:** This is the tangent of the line's electrical length (measured in radians) [199]. It is not a "wave index".

---

## 7. Parameters of Microstrip Patch Antennas

For your final goal of designing a **16-element microstrip patch antenna array** at a chosen operating frequency ($f_r$), the dimensions of each patch are determined by Pozar's design equations [3, 4]:

1. **Width ($W$):**
   $$W = \frac{c}{2f_r} \sqrt{\frac{2}{\epsilon_r + 1}}$$
2. **Effective Dielectric Constant ($\epsilon_{eff}$):** Due to fringing fields in the air above the substrate [254, 255]:
   $$\epsilon_{eff} = \frac{\epsilon_r + 1}{2} + \frac{\epsilon_r - 1}{2} \left[1 + 12 \frac{h}{W}\right]^{-1/2}$$
3. **Length Extension ($\Delta L$):** Accounts for physical patch length extension due to fringing fields [4]:
   $$\Delta L = 0.412 h \frac{(\epsilon_{eff} + 0.3)(W/h + 0.264)}{(\epsilon_{eff} - 0.258)(W/h + 0.8)}$$
4. **Physical Length ($L$):**
   $$L = L_{eff} - 2\Delta L \quad \text{where } L_{eff} = \frac{c}{2f_r \sqrt{\epsilon_{eff}}}$$

---

## 8. Reflection Coefficient ($\Gamma$), VSWR, and S-Parameters

### **The Reflection Coefficient ($\Gamma$)**
The voltage reflection coefficient ($\Gamma$ or $\rho$) is the complex ratio of the reflected voltage wave ($V^-$) to the incident voltage wave ($V^+$) [181, 670]:
$$\Gamma = \frac{V^-}{V^+} = \frac{Z_A - Z_0}{Z_A + Z_0}$$

### **Why do we use $20 \log_{10}$ for Voltages?**
Power is proportional to the square of voltage ($P \propto V^2$) [654, 660]. 
* When calculating power ratios in decibels, the formula is $10 \log_{10}(P_2/P_1)$ [187, 648].
* Since the power reflection ratio is $|\Gamma|^2$, expressing this in decibels yields [687]:
  $$\text{Return Loss (dB)} = 10 \log_{10}|\Gamma|^2 = 20 \log_{10}|\Gamma|$$
* Therefore, we use **$20 \log_{10}$** for voltage ratios (like $\Gamma$ or scattering parameters like $S_{11}$) to directly represent the corresponding power ratios in decibels [307, 660, 687]!

### **Correcting the VSWR, Return Loss, and Reflected Power Relationship**
In your note, you mixed up the values for VSWR, Return Loss, and Reflected Power. The mathematically precise relationships for standard matching thresholds are as follows:

| Standard Target | Voltage Reflection ($|\Gamma|$) | Power Reflected ($|\Gamma|^2$) | Return Loss (dB) | VSWR |
| :--- | :--- | :--- | :--- | :--- |
| **A: VSWR = 1.5** | $0.20$ | **$4.00\%$** | $-13.98\ \text{dB}$ | **$1.50$** |
| **B: Return Loss = -10 dB** | $0.316$ | **$10.00\%$** | **$-10.00\ \text{dB}$** | $1.92$ |
| **C: Reflected Power = 11.11%** | **$0.333$ (or $1/3$)** | **$11.11\%$ (or $1/9$)** | $-9.54\ \text{dB}$ | **$2.00$** |

* **Correction:** If you target a maximum allowable reflected power of $11.11\%$ ($|\Gamma| = 0.33$), your **VSWR is 2.0**, and your Return Loss is **$-9.54\ \text{dB}$** (often rounded to $-10\ \text{dB}$ in practice). If you want a **VSWR of 1.5**, you are restricting reflected power to only **$4\%$**, which corresponds to a much tighter Return Loss of **$-14\ \text{dB}$**.

### **$S_{11}$ and Impedance Bandwidth**
In a 1-port antenna network, the input scattering parameter $S_{11}$ is identical to the input reflection coefficient $\Gamma$ [307, 702]. 
When plotted as magnitude in dB ($20 \log_{10}|S_{11}|$) over frequency, it dips to a minimum at resonance [15, 639]. The **impedance bandwidth** is defined as the frequency range over which the reflection coefficient remains below a specified threshold (typically **$-10\ \text{dB}$**, corresponding to $\text{VSWR} \le 1.92$, or **$-9.54\ \text{dB}$**, corresponding to $\text{VSWR} \le 2.0$) [7, 639].
