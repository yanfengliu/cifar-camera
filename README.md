# CIFAR_camera
Project 2 for Advanced Embedded System class at University of Nebraska-Lincoln

Special thanks to [Jay Carlson](https://github.com/jaydcarlson), my instructor and mentor for this awesome class! 

Parts:
* STM32F4-discovery
* OV7670
* nRF24L01

Specification:
* Captures image in real time
* Classifies captured image upon button press, using pre-trained CIFAR-10 neural network
* Transmits classification result to another device via nRF24L01
