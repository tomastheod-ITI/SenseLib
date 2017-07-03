# SenseLib
__SenseLib__ is a C# library for data acquisition, processing and storage. Developed for the needs of the [ICT4Life project](http://www.ict4life.eu/), it can acquire data from the following sensors:
-	Kinect
-	Zenith camera
-	Microsoft band
-	Hexiwear band
-	UPM band
-	Binary
-	WSN (Wireless Sensor Network)

The library consists of three main namespaces: `DataFrames`, `Acquisition` and `Utils`.

Inside the `DataFrames` namespace, there are appropriate containers for each implemented sensor, so that a single frame from the sensor can be held in memory for further processing.

The `Acquisition` namespace contains a set of methods for each sensor needed to interact with the sensor. Despite each sensor requiring a slightly different set of methods, the following methods are common to all sensors:

| Method            | Description                                  |
| ----------------- | -------------------------------------------- |
| InitializeSensor  | Initializes sensor parameters.               |
| IsSensorAvailable | Checks if the sensor is connected.           |
| GetFrame          | Requests a new frame from the sensor.        |
| FrameToBytes      | Converts the sensor frame into a byte array. |
| BytesToFrame      | Converts a byte array back into a frame.     |

The `Utils` namespace offers a plethora of different functionalities such as __data compression__, __image scaling__, __video creation__ and __database storage__. The general outline of this namespace is the following:

| Class / Enum      | Description                                                                        |
| ----------------- | ---------------------------------------------------------------------------------- |
| SensorType        | An enumeration of the supported sensors.                                           |
| SensorUtils       | Sensor-related functions, such as enumerating the sensors available to the system. |
| Serialization     | Methods for serializing, deep copying and packaging of arbitrary objects.          |
| ConnectionInfo    | Methods regarding network connectivity, such as which system ports are in use.     |
| Validation        | Methods for validating that a number or region is within the supplied limits.      |
| DataIO            | Methods for data storage and retrieval.                                            |
| FrameOperations   | Methods for manipulating images and coordinates.                                   |

The __MSBandSenseLib__ is responsible for acquiring data from the Microsoft band and is distributed separately, because it targets Universal Windows, while __SenseLib__ targets .NET Framework 4.5. Both libraries are compiled for the x64 architecture.

### Dependencies
__SenseLib__ references the following libraries:
- Emgu CV 3.1.0
- InTheHand 3.5
- Kinect 2.0
- Infer.NET 2.6
- MongoDB C# Driver 2.4.3
- ZstdNet 1.0

__MSBandSenseLib__ references:
- Microsoft Band 1.3

### License
This project is licensed under the [MIT License](https://choosealicense.com/licenses/mit/).

### Contributors
This project is a collaborative effort by:
- Information Technologies Institute of Centre for Research and Technology Hellas (CERTH-ITI)
- Technical University of Madrid (UPM)
- Maastricht University (UM)
- Artica Telemedicina (Artica)

### Disclaimer
All mentioned product names and brands are property of their respective owners and are used for identification purposes only. Use of these product names and brands does not imply endorsement.

### Examples
The following code snippet demonstrates the use of __SenseLib__ in order to acquire data from Kinect, draw the detected skeletons as well as the text “Person” above each skeleton onto the acquired color image, and display the final color image on screen. It is assumed that the user has created a C# WPF project named `SenseLibTest` and added two buttons (`btnBeginCapture`, `btnStopCapture`) and an image control (`imgDisplay`) to the user interface.

```c#
using SenseLib.Acquisition.Kinect;
using SenseLib.DataFrames.Kinect;
using SenseLib.Utils;
using System;
using System.Windows;

namespace SenseLibTest
{
    public partial class MainWindow : Window
    {
        private KinectSenseLib kinect;
        private KinectFrame frame;

        private bool pause = true;

        public MainWindow()
        {
            InitializeComponent();
        }

        private async void btnBeginCapture_Click(object sender, RoutedEventArgs e)
        {
            if (kinect == null)
            {
                // Initialize Kinect acquisition.
                kinect = new KinectSenseLib();
                kinect.InitializeSensor(new KinectSelection(true));
            }

            // Check that the sensor is available. Wait 5 seconds for a response.
            if (kinect.IsSensorAvailable(5000))
            {
                kinect.OpenSensor();
                pause = false;
            }
            else
            {
                pause = true;
            }

            // While the user has not pressed the stop button...
            while (pause == false)
            {
                // Create a new thread in order to get data.
                await System.Threading.Tasks.Task.Run(() =>
                {
                    // Request a new frame.
                    frame = kinect.GetFrame();

                    // Draw any detected skeletons onto the acquired color image
                    // using the raw (unfiltered) joint positions and a dot size of 6.
                    frame.ColorImage = 
                          FrameOperations.Transform.SkeletonToColor(frame, 
                          FrameOperations.ColorTypes.Bgra, false, 6);

                    // Draw the text 'Person' above each tracked skeleton 
                    // onto the acquired color image.
                    frame.ColorImage = 
                          FrameOperations.Transform.TextToColor(frame.ColorImage, frame, 
                          "Person ", false);

                    // Let the main thread perform the actual drawing.
                    Application.Current.Dispatcher.BeginInvoke((Action)(() =>
                        // Convert the color image into a bitmap and display it.
                        imgDisplay.Source =
                                   FrameOperations.Convert.BytesToBitmap(frame.ColorImage, 
                                   frame.WidthColor, frame.HeightColor, 
                                   System.Windows.Media.PixelFormats.Bgra32)));
                });
            }

            kinect.CloseSensor();
        }

        private void btnStopCapture_Click(object sender, RoutedEventArgs e)
        {
            pause = true;
        }
    }
}
```
