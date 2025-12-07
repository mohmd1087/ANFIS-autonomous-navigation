# ANFIS-Based Autonomous Navigation System Using CoppeliaSim + MATLAB Remote API

This project implements a fully simulated autonomous navigation system for a differential-drive mobile robot (Pioneer P3-DX) using an **Adaptive Neuro-Fuzzy Inference System (ANFIS)**.  
The controller learns human-like navigation behavior from teleoperation data and later applies it autonomously inside a maze-like environment with static and dynamic obstacles.

Simulation, robot control, and machine learning are integrated using the **CoppeliaSim Remote API** and **MATLAB ANFIS Toolbox**.

---

## 🚀 Project Overview

The system operates in three phases:

### **1️⃣ Data Collection (Human Teleoperation)**
- Robot is driven manually via keyboard.
- Ultrasonic readings + goal direction + wheel speeds are recorded.
- Multiple teleoperation runs generate a diverse training dataset.

### **2️⃣ ANFIS Model Training**
Two ANFIS models are trained in MATLAB:

- **Left wheel controller**
- **Right wheel controller**

Inputs:
- S1: left sonar cluster  
- S2: front sonar cluster  
- S3: right sonar cluster  
- heading_error: orientation difference to goal

Outputs:
- vL: left wheel velocity  
- vR: right wheel velocity  

### **3️⃣ Autonomous Navigation**
MATLAB runs a closed-loop ANFIS-based controller:
- Read sensors → Preprocess → Evaluate ANFIS → Send wheel commands → Repeat  
- Robot autonomously avoids walls and reaches the goal.

---
---

## 🛠 Software Requirements

- MATLAB (R2020+ recommended)  
- Fuzzy Logic Toolbox  
- CoppeliaSim EDU (v4.3+)  
- Remote API bindings for MATLAB  

---

## 🔧 Setup Instructions

### **1. Configure CoppeliaSim**
- Load Pioneer P3-DX model
- Disable its default child script
- Add maze walls using cuboids (static + respondable)
- Add moving obstacle (oscillating cylinder)
- Add goal dummy
- Add third-person chase camera (Lua script)

### **2. Enable Remote API**
In a child script:

```lua
simRemoteApi.start(19999)


addpath('remote_api');
addpath('scripts');
addpath('fis_models');
