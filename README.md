# ShippingElementDetector
Idea/algorithm created and implemented by Allen Wu, code provided by FTC 18225 High Definition

NOTE: EXTRA CHANGES WILL NEED TO BE MADE TO ACCOMODATE A RED OR BLUE TEAM SHIPPING ELEMENT. FOR EASIEST USE OF THIS, PLEASE CONSIDER CHANGING THE COLOR OF YOUR TEAM SHIPPING ELEMENT IF IT HAS RED OR BLUE ON IT.

Follow the following steps to use this in your own code:
1. Copy the Detector.java and DetectorTest.java class into your code base
2. Make sure your robot is positioned as it would in the start of autonomous
3. Run the DetectorTest.java op-mode and press 'A' to capture a new image
4. Through command line/terminal, type in the following command: [COMMAND]
5. While your camera is off, adjust and press 'A' again
6. Find the image boundaries (estimate) to make only two of the markers visible
7. Use these image boundaries in your autonomous op-mode
8. Repeat Steps (3) to (7) again for each autonomous program

Make sure you change WEBCAM_CONFIG_NAME inside Detector.java to match your webcam name in the configuration.
