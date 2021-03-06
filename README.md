# Pen Glove

Source code for a data glove built with an MPU6050 sensor and Arduino Uno to be used as a virtual pen.

Capabilities of the glove include:
* Allow the user to write or draw by moving their hand in the air
* Transfer data to a computer through Mosquitto MQTT 
* Recognize the input with the Tesseract OCR integration (in progress; can only detect single characters and letters)

---

For a mini demonstration of the first prototype, visit the following YouTube link: 

[![Arduino Glove](https://img.youtube.com/vi/E4OcadS2zEE/0.jpg)](https://www.youtube.com/watch?v=E4OcadS2zEE)

This version only involves the acquision/interpretation of accelerometer and gyroscope data, and display of the input in real time with a Processing-based GUI.


