

# HiKVision Camera Drivers

This project provides a set of utility functions for interacting with cameras using the MvCamera SDK. It includes functionality for image capturing, device configuration, parameter management, IEEE 1588, action commands, etc.


## **Dependencies**

- **C++ Compiler**: Ensure you have a C++ compiler installed (e.g., `g++`).
- **MvCamera SDK**: [机器视觉工业相机客户端MVS V3.0.1 (Linux)](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0)
- **Make**: To build the project using the provided Makefile.
- **HikVision API**: [HikVisionAPI](https://github.com/leoxiang66/hikvision_camera_api)
---


---



## **Build Instructions**

### **1. Build the Project**

Clone this repo into a ros2 workspace `hik_ws`

```sh
git clone https://github.com/leoxiang66/hikvision_ros2_driver.git hik_ws/src/hikvision_ros2_driver
```

```sh
cd hik_ws/src/hikvision_ros2_driver
./build.sh
```

---

### **2. Run the Program**


```sh
ros2 run hikvision_ros2_driver image_publisher
```

---

## **License**

MIT License

Copyright (c) [2024] [XIANG Tao]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

## **Contact**

For questions, issues, or contributions, feel free to open an issue.

