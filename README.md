# CMPUT312-LAB-03
Our project structure is defined as follows:
- **bottom-layer - robot_core**: This layer interacts directly with ev3dev2 APIs and provides essential control functions needed to operate the robot arm. It contains the TCP server and TCP client classes to facilitate communication between the EV3 computer and the host PC running the vision model.
- **middle-layer - kinematics**: This layer implements all forward and inverse kinematic methods.
- **top-layer - lab_tasks**: This layer is used to implement tasks directly from our lab assignments.

```
CMPUT312-LAB-03/
├── VSMaterial/                        # visual servoing testing modules 
│   ├── __init__.py
│   ├── color_tracking_hsv.py          # HSV-based color tracking tuning program
│
├── kinematics/                        # mid-layer kinematics algorithms
│   ├── __init__.py
│   ├── compute_Jacobian.py            # Jacobian matrix computation
│   ├── forward.py                     # forward kinematics
│   ├── helper.py                      # helper functions
│   ├── inverse_numerical_broyden.py   # Broyden’s method
│   └── inverse_numerical_newton.py    # Newton’s method
│
├── robot_core/                        # bottom-layer robot drivers & networking
│   ├── __init__.py
│   ├── arm_client.py                  # client communication interface
│   ├── arm_driver.py                  # robot arm low-level driver
│   ├── arm_server.py                  # server communication interface
│   ├── arm_tracker.py                 # robot pose & movement tracking
│   └── network_settings.py            # network configuration parameters
│
├── tasks/...                          # top-layer lab tasks
├── main_client.py                     # main program entry for EV3 computer
└── main_host.py                       # main program entry for host PC
```