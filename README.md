# MADE-Axis: A Modular Actuated Device to Embody the Axis of a Data Dimension
<img src="https://user-images.githubusercontent.com/36180947/142817949-1d690bd1-7f1a-4a95-86bd-95a6675367b5.jpg" height="250" /> <img src="https://user-images.githubusercontent.com/36180947/142817969-ae63db5a-e2d8-4959-bd10-08e8ad4f1a5d.jpg" height="250" />
<br>
The MADE-Axis is a modular, wireless, and re-configurable device that can be used in a variety of different data visualisation scenarios. It is designed to offer affordances for data interaction in terms of input, output, and composability.

Our work demonstrated the MADE-Axis in the following applications:

*As a HSV colour picker* <br>
<img src="https://user-images.githubusercontent.com/36180947/142818733-c5a9c7a9-8b1d-4290-9daf-5af71e67c14f.png" height="225" />

*As a controller for time series visualisation manipulation* <br>
<img src="https://user-images.githubusercontent.com/36180947/142818736-74c33421-aefe-40bc-87b5-4f0a46484a4a.PNG" height="250" />

*As a virtual reality controller* <br>
<img src="https://user-images.githubusercontent.com/36180947/142817994-efd02ffc-8db1-4057-8b75-0cf1cf9cb9d6.PNG" height="250" />

*As an embodied augmented reality data visualisation system* <br>
<img src="https://user-images.githubusercontent.com/36180947/142817959-fcee1366-de3b-48d2-988c-e08bc528a9d9.png" height="300" />

This Github repository contains the code used for the publication titled *The MADE-Axis: A Modular Actuated Device to Embody the Axis of a Data Dimension*, which was published at ACM ISS 2021. A pre-print of the paper can be found [here](https://ialab.it.monash.edu/~dwyer/papers/madeaxis.pdf).

## Repository
This repository currently contains the following files:

 - Gerber files for the MADE-Axis' PCB design
 - Arduino code for the MADE-Axis
 - A Unity project for the demonstrated augmented reality system

#### AR System
The Unity AR project was developed using the following plugins:
 - [ImAxes](https://github.com/MaximeCordeil/ImAxes)
 - Mixed Reality Toolkit
 - Photon Unity Networking
 - ViconUnityPlugin
 - [HoloLens 2 QR code tracking by Joost van Schaik](https://localjoost.github.io/Positioning-QR-codes-in-space-with-HoloLens-2-building-a-%27poor-man%27s-Vuforia%27/)

The project contains two scenes of note: *ServerScene* and *HoloLensScene*.

*ServerScene* is meant to be run on a desktop PC and serves as an intermediary between the input devices and the HL2s. Physical positions and orientations of the MADE-Axes are tracked using a Vicon motion capture system and the Vicon Tracker software on the PC. The MADE-Axes themselves are connected to the same PC via bluetooth, and sends values for the sliders, rotary dial, and push button. All of these values are collated by the *ServerScene* and are broadcasted to the HL2s using Photon Unity Networking.

*HoloLensScene* is meant to be deployed on one or more HoloLens 2s. This functionally acts as an "observer" which renders data visualisations in the physical space. No input functionality of the HL2 is used. The aforementioned values sent by the *ServerScene* are processed, and the resulting visualisations are overlaid on top of the physical devices.

The coordinate systems of the Vicon and HL2s are synchronised using a QR code. The position of the QR code is known to the Vicon as a tracked object. The QR code is also tracked by the HL2 using its QR code detection capabilities. The QR code then acts as a shared position in 3D space which is used to synchronise the coordinate spaces.


## Publication
To cite our work, please use the following *BibTeX*:

> @article{10.1145/3488546,
author = {Smiley, Jim and Lee, Benjamin and Tandon, Siddhant and Cordeil, Maxime and Besan\c{c}on, Lonni and Knibbe, Jarrod and Jenny, Bernhard and Dwyer, Tim},
title = {The MADE-Axis: A Modular Actuated Device to Embody the Axis of a Data Dimension},
year = {2021},
issue_date = {November 2021},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
volume = {5},
number = {ISS},
url = {https://doi.org/10.1145/3488546},
doi = {10.1145/3488546},
journal = {Proc. ACM Hum.-Comput. Interact.},
month = {nov},
articleno = {501},
numpages = {23},
keywords = {data visualization, embodied interfaces}
}

## Video Previews
Please see the following videos if you would like to see the applications in action.

*Video demo* <br>
[![](https://img.youtube.com/vi/ILZlecsvUbw/0.jpg)](https://www.youtube.com/watch?v=ILZlecsvUbw)

*Presentation @ ACM ISS 2021* <br>
[![](https://img.youtube.com/vi/UWpv0TJBqAI/0.jpg)](https://www.youtube.com/watch?v=UWpv0TJBqAI)
