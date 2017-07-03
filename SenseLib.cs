
////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                            ////
////   Copyright (c) 2016 - present, CERTH-ITI, UPM, UM, Artica                                 ////
////                                                                                            ////
////////////////////////////////////////////////////////////////////////////////////////////////////


using Emgu.CV;
using Emgu.CV.Structure;
using InTheHand.Net.Sockets;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using MicrosoftResearch.Infer.Maths;
using MongoDB.Bson;
using MongoDB.Driver;
using SenseLib.DataFrames.Binary;
using SenseLib.DataFrames.HexiwearBand;
using SenseLib.DataFrames.Kinect;
using SenseLib.DataFrames.MSBand;
using SenseLib.DataFrames.UPMBand;
using SenseLib.DataFrames.WSN;
using SenseLib.DataFrames.Zenith;
using SenseLib.Utils;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using ZstdNet;


namespace SenseLib
{

    namespace DataFrames
    {

        namespace Kinect
        {
            /// <summary>Represents a Kinect skeleton joint.</summary>
            [Serializable, DataContract]
            public struct SkeletonJoint
            {
                /// <summary>The x axis coordinate.</summary>
                [DataMember]
                public float x;
                /// <summary>The y axis coordinate.</summary>
                [DataMember]
                public float y;
                /// <summary>The z axis coordinate.</summary>
                [DataMember]
                public float z;
                /// <summary>The tracking confidence.</summary>
                [DataMember]
                public int confidence;

                public override string ToString()
                {
                    return x + " " + y + " " + z + " " + confidence;
                }
            }

            /// <summary>Represents a Kinect skeleton joint.</summary>
            [Serializable, DataContract]
            public struct Skeleton
            {
                [DataMember]
                public SkeletonJoint[] jointsRaw;               // raw joint positions in world coordinates
                [DataMember]
                public SkeletonJoint[] jointsRawColor;          // raw joint positions in color space
                [DataMember]
                public SkeletonJoint[] jointsRawGray;           // raw joint positions in depth space
                [DataMember]
                public SkeletonJoint[] jointsFiltered;          // filtered joint positions in world coordinates
                [DataMember]
                public SkeletonJoint[] jointsFilteredColor;     // filtered joint positions in color space
                [DataMember]
                public SkeletonJoint[] jointsFilteredGray;      // filtered joint positions in depth space
            }

            [Serializable, DataContract]
            public struct Body
            {
                [DataMember]
                public float leanFB;                            // forwards / backwards leaning
                [DataMember]
                public float leanLR;                            // left / right leaning
                [DataMember]
                public int leanConfidence;                      // leaning tracking confidence
                [DataMember]
                public bool isTracked;                          // if Kinect has found a body
                [DataMember]
                public bool firstTrack;                         // if the person is tracked for the first time
                [DataMember]
                public Skeleton skeletonData;                   // body skeleton data
            }

            [Serializable, DataContract]
            public struct FaceBox
            {
                public int Top;                                 // top position
                public int Bottom;                              // bottom position
                public int Left;                                // left position
                public int Right;                               // right position
            }

            [Serializable, DataContract]
            public class KinectFrame
            {
                private string sensorID;                        // the unique Kinect ID

                private DateTime timeStamp;                     // the frame acquisition time

                private byte[] colorImage;                      // the color image (BGRA)
                private byte[] depthImage;                      // the depth image
                private byte[] infraImage;                      // the infrared image
                private byte[] bodyImage;                       // the body index image 
                private Body[] bodyFrame;                       // the body data

                private FaceBox[] faceBoundingBoxColor;         // the face bounding box in color space
                private FaceBox[] faceBoundingBoxGray;          // the face bounding box in depth space

                public int WidthColor;
                public int HeightColor;
                public int WidthGray;
                public int HeightGray;

                private const int maxBodies = 6;
                private const int maxJoints = 25;


                public string SensorID
                {
                    get { return sensorID; }
                    set { sensorID = value; }
                }
                public DateTime Timestamp
                {
                    get { return timeStamp; }
                    set { timeStamp = value; }
                }
                public byte[] ColorImage
                {
                    get { return colorImage; }
                    set { colorImage = value; }
                }
                public byte[] DepthImage
                {
                    get { return depthImage; }
                    set { depthImage = value; }
                }
                public byte[] InfraImage
                {
                    get { return infraImage; }
                    set { infraImage = value; }
                }
                public byte[] BodyImage
                {
                    get { return bodyImage; }
                    set { bodyImage = value; }
                }
                public Body[] BodyFrame
                {
                    get { return bodyFrame; }
                    set { bodyFrame = value; }
                }
                public FaceBox[] FaceBoundingBoxColor
                {
                    get { return faceBoundingBoxColor; }
                    set { faceBoundingBoxColor = value; }
                }
                public FaceBox[] FaceBoundingBoxGray
                {
                    get { return faceBoundingBoxGray; }
                    set { faceBoundingBoxGray = value; }
                }
                public int MaxBodies
                {
                    get { return maxBodies; }
                }
                public int MaxJoints
                {
                    get { return maxJoints; }
                }

                // Initialization
                public KinectFrame(float scaleColor = 1.0f, float scaleGray = 1.0f)
                {
                    sensorID = "";

                    timeStamp = new DateTime();

                    WidthColor = (int)(1920 * scaleColor);
                    HeightColor = (int)(1080 * scaleColor);
                    WidthGray = (int)(512 * scaleGray);
                    HeightGray = (int)(424 * scaleGray);

                    colorImage = new byte[WidthColor * HeightColor * 4];
                    depthImage = new byte[WidthGray * HeightGray * 2];
                    infraImage = new byte[WidthGray * HeightGray * 2];
                    bodyImage = new byte[WidthGray * HeightGray];

                    bodyFrame = new Body[MaxBodies];
                    faceBoundingBoxColor = new FaceBox[MaxBodies];
                    faceBoundingBoxGray = new FaceBox[MaxBodies];
                    for (int i = 0; i < MaxBodies; i++)
                    {
                        bodyFrame[i].leanFB = 0;
                        bodyFrame[i].leanLR = 0;
                        bodyFrame[i].isTracked = false;
                        bodyFrame[i].firstTrack = false;

                        bodyFrame[i].skeletonData.jointsRaw = new SkeletonJoint[MaxJoints];
                        bodyFrame[i].skeletonData.jointsRawColor = new SkeletonJoint[MaxJoints];
                        bodyFrame[i].skeletonData.jointsRawGray = new SkeletonJoint[MaxJoints];
                        bodyFrame[i].skeletonData.jointsFiltered = new SkeletonJoint[MaxJoints];
                        bodyFrame[i].skeletonData.jointsFilteredColor = new SkeletonJoint[MaxJoints];
                        bodyFrame[i].skeletonData.jointsFilteredGray = new SkeletonJoint[MaxJoints];

                        faceBoundingBoxColor[i].Bottom = -1;
                        faceBoundingBoxColor[i].Left = -1;
                        faceBoundingBoxColor[i].Right = -1;
                        faceBoundingBoxColor[i].Top = -1;

                        faceBoundingBoxGray[i].Bottom = -1;
                        faceBoundingBoxGray[i].Left = -1;
                        faceBoundingBoxGray[i].Right = -1;
                        faceBoundingBoxGray[i].Top = -1;
                    }
                }
            }
        }

        namespace Zenith
        {
            [Serializable]
            public class ZenithFrame
            {
                private DateTime timeStamp;
                private byte[] colorData;
                public int Width;
                public int Height;

                public DateTime Timestamp
                {
                    get { return timeStamp; }
                    set { timeStamp = value; }
                }
                public byte[] ColorData
                {
                    get { return colorData; }
                    set { colorData = value; }
                }
				

                // Initialization
                public ZenithFrame(int frameWidth = 1056, int frameHeight = 1056)
                {
                    Width = frameWidth;
                    Height = frameHeight;

                    timeStamp = new DateTime();
                    colorData = new byte[frameWidth * frameHeight * 3];
                }

            }
        }

        namespace Binary
        {
            [Serializable]
            public class BinaryFrame
            {
                public string BinarySensorOutput;
                public DateTime TimeStamp;
				

                public BinaryFrame()
                {
                    BinarySensorOutput = "";
                    TimeStamp = new DateTime();
                }
            }
        }

        namespace WSN
        {
            [Serializable]
            public class WSNFrame
            {
                public string WSNOutput;
				
				

                public WSNFrame()
                {
                    WSNOutput = "";
                }
            }
        }

        namespace HexiwearBand
        {
            [Serializable]
            public class HexiwearBandFrame
            {
                public string HexiwearBandOutput;


                public HexiwearBandFrame()
                {
                    HexiwearBandOutput = "";
                }
            }
        }

        namespace UPMBand
        {
            [Serializable]
            public class UPMBandFrame
            {
                public string UPMBandOutput;


                public UPMBandFrame()
                {
                    UPMBandOutput = "";
                }
            }
        }

        namespace MSBand
        {
            [DataContract]
            public struct HeartRateDataBand
            {
                private int data;
                private DateTime timeStamp;

                [DataMember]
                public int Data { get { return data; } set { data = value; } }
                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public struct SkinTemperatureDataBand
            {
                private double data;
                private DateTime timeStamp;

                [DataMember]
                public double Data { get { return data; } set { data = value; } }
                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public struct CaloriesDataBand
            {
                private long dataTotal;
                private long dateToday;
                private DateTime timeStamp;

                [DataMember]
                public long DataTotal { get { return dataTotal; } set { dataTotal = value; } }
                [DataMember]
                public long DataToday { get { return dateToday; } set { dateToday = value; } }
                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public struct GyroscopeDataBand
            {
                private double accelerationX;
                private double accelerationY;
                private double accelerationZ;
                private double angularVelocityX;
                private double angularVelocityY;
                private double angularVelocityZ;

                private DateTime timeStamp;

                [DataMember]
                public double AccelerationX { get { return accelerationX; } set { accelerationX = value; } }
                [DataMember]
                public double AccelerationY { get { return accelerationY; } set { accelerationY = value; } }
                [DataMember]
                public double AccelerationZ { get { return accelerationZ; } set { accelerationZ = value; } }
                [DataMember]
                public double AngularVelocityX { get { return angularVelocityX; } set { angularVelocityX = value; } }
                [DataMember]
                public double AngularVelocityY { get { return angularVelocityY; } set { angularVelocityY = value; } }
                [DataMember]
                public double AngularVelocityZ { get { return angularVelocityZ; } set { angularVelocityZ = value; } }

                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public struct PedometerDataBand
            {
                private long stepsToday;
                private long stepsTotal;
                private DateTime timeStamp;

                [DataMember]
                public long StepsToday { get { return stepsToday; } set { stepsToday = value; } }
                [DataMember]
                public long StepsTotal { get { return stepsTotal; } set { stepsTotal = value; } }
                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public struct DistanceDataBand
            {
                private long distanteToday;
                private double pace;
                private double speed;
                private DateTime timeStamp;

                [DataMember]
                public long DistanteToday { get { return distanteToday; } set { distanteToday = value; } }
                [DataMember]
                public double Pace { get { return pace; } set { pace = value; } }
                [DataMember]
                public double Speed { get { return speed; } set { speed = value; } }
                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public struct GalvanicSkinResponse
            {
                private long gsr;
                private DateTime timeStamp;

                [DataMember]
                public long Gsr { get { return gsr; } set { gsr = value; } }
                [DataMember]
                public DateTime DataTimeStamp { get { return timeStamp; } set { timeStamp = value; } }
            }

            [DataContract]
            public class MSBandFrame
            {
                [DataMember]
                public string SensorID { get; set; }

                [DataMember]
                public DateTime TimeStamp { get; set; }

                [DataMember]
                public HeartRateDataBand HeartRateData { get; set; }
                [DataMember]
                public SkinTemperatureDataBand SkinTemperatureData { get; set; }
                [DataMember]
                public CaloriesDataBand CaloriesData { get; set; }
                [DataMember]
                public GyroscopeDataBand GyroscopeData { get; set; }
                [DataMember]
                public PedometerDataBand PedometerData { get; set; }
                [DataMember]
                public DistanceDataBand DistanceData { get; set; }
                [DataMember]
                public GalvanicSkinResponse GalvanicSkinResponseData { get; set; }
            }
        }

    }
    
    namespace Acquisition
    {

        namespace Kinect
        {
            /// <summary>Denotes the data streams to be used for acquisition.</summary>
            [Serializable]
            public struct KinectSelection
            {
                public bool colorImage;
                public bool depthImage;
                public bool infraImage;
                public bool bodyImage;
                public bool bodyFrame;
                public bool faceBoundingBox;

                /// <summary>Initializes all data streams with the given value.</summary>
                public KinectSelection(bool init)
                {
                    colorImage = init;
                    depthImage = init;
                    infraImage = init;
                    bodyImage = init;
                    bodyFrame = init;
                    faceBoundingBox = init;
                }

                /// <summary>Initializes the data streams with the given boolean array.</summary>
                public KinectSelection(bool[] init)
                {
                    colorImage = init[0];
                    depthImage = init[1];
                    infraImage = init[2];
                    bodyImage = init[3];
                    bodyFrame = init[4];
                    faceBoundingBox = init[5];
                }
            }

            /// <summary>Contains the Kinect acquisition methods.</summary>
            public class KinectSenseLib
            {
                private const int sleepTime = 4;

                public KinectFrame currentFrame;
                public KinectSelection selection;

                private KinectSensor sensor;
                private CoordinateMapper mapper;

                private MultiSourceFrameReader readerMS;
                private FaceFrameSource[] faceFrameSources;
                private FaceFrameReader[] faceFrameReaders;
                private FaceFrameResult[] faceFrameResults;

                private Microsoft.Kinect.Body[] bodies;
                private bool[] isBodyTracked;

                private readonly object frameLock1;
                private readonly object frameLock2;

                private DateTime readerTime;
                private DateTime sendTime;

                private DateTime time;

                private int defaultWidthColor;
                private int defaultHeightColor;
                private int defaultWidthGray;
                private int defaultHeightGray;

                private float scalingColor;
                private float scalingGray;

                private bool scaleColor;
                private bool scaleGray;

                private int compressionColor;
                private int compressionGray;

                private byte[] colorImage;
                private ushort[] tmpDepthImage;
                private ushort[] tmpInfraImage;

                private byte[] depthImage;
                private byte[] infraImage;
                private byte[] bodyImage;

                private DataFrames.Kinect.Body[] bodyFrame;
                private DataFrames.Kinect.FaceBox[] faceBoxColor;
                private DataFrames.Kinect.FaceBox[] faceBoxInfra;

                private byte[] scaledColor;
                private byte[] scaledDepth;
                private byte[] scaledInfra;
                private byte[] scaledBody;

                private DataFrames.Kinect.Body[] scaledBodyFrame;
                private DataFrames.Kinect.FaceBox[] scaledFaceBoxColor;
                private DataFrames.Kinect.FaceBox[] scaledFaceBoxGray;

                private int readerCounter;
                private int sendCounter;

                private SkeletonJoint[][] lastFiltered;
                private double[][,] ErrorCovPost;


                /// <summary>Library initializer.</summary>
                public KinectSenseLib()
                {
                    frameLock1 = new object();
                    frameLock2 = new object();
                }

                /// <summary>Initializes the library parameters and opens the sensor.</summary>
                public void InitializeSensor(KinectSelection ks, float ColorScale = 1.0f, float GrayScale = 1.0f, int ColorCompression = 80, int GrayCompression = 0)
                {
                    currentFrame = new KinectFrame(ColorScale, GrayScale);
                    selection = ks;

                    scalingColor = ColorScale;
                    scalingGray = GrayScale;

                    scaleColor = (int)ColorScale != 1;
                    scaleGray = (int)GrayScale != 1;

                    compressionColor = ColorCompression;
                    compressionGray = GrayCompression;

                    sensor = KinectSensor.GetDefault();
                    mapper = sensor.CoordinateMapper;

                    readerMS = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | 
                                                                 FrameSourceTypes.Depth | 
                                                                 FrameSourceTypes.Infrared | 
                                                                 FrameSourceTypes.Body | 
                                                                 FrameSourceTypes.BodyIndex);

                    readerMS.MultiSourceFrameArrived += reader_MultiSourceFrameArrived;

                    FaceFrameFeatures faceFrameFeatures =
                        FaceFrameFeatures.BoundingBoxInColorSpace |
                        FaceFrameFeatures.BoundingBoxInInfraredSpace |
                        FaceFrameFeatures.PointsInColorSpace |
                        FaceFrameFeatures.PointsInInfraredSpace
                    ;

                    isBodyTracked = new bool[currentFrame.MaxBodies];
                    bodies = new Microsoft.Kinect.Body[currentFrame.MaxBodies];

                    faceFrameSources = new FaceFrameSource[currentFrame.MaxBodies];
                    faceFrameReaders = new FaceFrameReader[currentFrame.MaxBodies];
                    for (int i = 0; i < currentFrame.MaxBodies; i++)
                    {
                        faceFrameSources[i] = new FaceFrameSource(sensor, 0, faceFrameFeatures);
                        faceFrameReaders[i] = faceFrameSources[i].OpenReader();
                        if (faceFrameReaders[i] != null)
                        {
                            faceFrameReaders[i].FrameArrived += reader_FaceFrameArrived;
                        }
                    }
                    faceFrameResults = new FaceFrameResult[currentFrame.MaxBodies];

                    readerTime = DateTime.Now;
                    sendTime = DateTime.Now;

                    time = DateTime.Now;

                    defaultWidthColor = sensor.ColorFrameSource.FrameDescription.Width;
                    defaultHeightColor = sensor.ColorFrameSource.FrameDescription.Height;
                    defaultWidthGray = sensor.DepthFrameSource.FrameDescription.Width;
                    defaultHeightGray = sensor.DepthFrameSource.FrameDescription.Height;

                    colorImage = new byte[defaultWidthColor * defaultHeightColor * 4];
                    depthImage = new byte[defaultWidthGray * defaultHeightGray * 2];
                    infraImage = new byte[defaultWidthGray * defaultHeightGray * 2];
                    bodyImage = new byte[defaultWidthGray * defaultHeightGray];

                    tmpDepthImage = new ushort[defaultWidthGray * defaultHeightGray];
                    tmpInfraImage = new ushort[defaultWidthGray * defaultHeightGray];

                    bodyFrame = new DataFrames.Kinect.Body[currentFrame.MaxBodies];
                    faceBoxColor = new FaceBox[currentFrame.MaxBodies];
                    faceBoxInfra = new FaceBox[currentFrame.MaxBodies];
                    for (int i = 0; i < currentFrame.MaxBodies; i++)
                    {
                        bodyFrame[i].leanFB = 0;
                        bodyFrame[i].leanLR = 0;
                        bodyFrame[i].isTracked = false;
                        bodyFrame[i].firstTrack = false;

                        bodyFrame[i].skeletonData.jointsRaw = new SkeletonJoint[currentFrame.MaxJoints];
                        bodyFrame[i].skeletonData.jointsRawColor = new SkeletonJoint[currentFrame.MaxJoints];
                        bodyFrame[i].skeletonData.jointsRawGray = new SkeletonJoint[currentFrame.MaxJoints];
                        bodyFrame[i].skeletonData.jointsFiltered = new SkeletonJoint[currentFrame.MaxJoints];
                        bodyFrame[i].skeletonData.jointsFilteredColor = new SkeletonJoint[currentFrame.MaxJoints];
                        bodyFrame[i].skeletonData.jointsFilteredGray = new SkeletonJoint[currentFrame.MaxJoints];

                        faceBoxColor[i].Bottom = -1;
                        faceBoxColor[i].Left = -1;
                        faceBoxColor[i].Right = -1;
                        faceBoxColor[i].Top = -1;

                        faceBoxInfra[i].Bottom = -1;
                        faceBoxInfra[i].Left = -1;
                        faceBoxInfra[i].Right = -1;
                        faceBoxInfra[i].Top = -1;
                    }

                    scaledColor = null;
                    scaledDepth = null;
                    scaledInfra = null;
                    scaledBody = null;

                    scaledBodyFrame = null;
                    scaledFaceBoxColor = null;
                    scaledFaceBoxGray = null;

                    readerCounter = 0;
                    sendCounter = 0;

                    lastFiltered = new SkeletonJoint[currentFrame.MaxBodies][];
                    ErrorCovPost = new double[currentFrame.MaxBodies][,];
                    for (int i = 0; i < currentFrame.MaxBodies; i++)
                    {
                        lastFiltered[i] = new SkeletonJoint[currentFrame.MaxJoints];
                        ErrorCovPost[i] = new double[currentFrame.MaxJoints, 3];
                        for (int j = 0; j < currentFrame.MaxJoints; j++)
                        {
                            ErrorCovPost[i][j, 0] = 0.01;
                            ErrorCovPost[i][j, 1] = 0.01;
                            ErrorCovPost[i][j, 2] = 0.01;
                        }
                    }

                    currentFrame.SensorID = sensor.UniqueKinectId;

                    OpenSensor();
                }

                #region *** Readers ***

                private void reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
                {
                    try
                    {
                        lock (frameLock1)
                        {
                            readerTime = DateTime.Now;
                            if (sendCounter > 0)
                            {
                                readerCounter++;
                            }

                            MultiSourceFrame ms_f = e.FrameReference.AcquireFrame();
                            if (ms_f != null)
                            {
                                if (selection.colorImage)
                                {
                                    ColorFrame color_f = ms_f.ColorFrameReference.AcquireFrame();
                                    using (color_f)
                                    {
                                        if (color_f != null)
                                        {
                                            color_f.CopyConvertedFrameDataToArray(colorImage, ColorImageFormat.Bgra);
                                        }
                                    }
                                }
                                if (selection.depthImage)
                                {
                                    DepthFrame depth_f = ms_f.DepthFrameReference.AcquireFrame();
                                    using (depth_f)
                                    {
                                        if (depth_f != null)
                                        {
                                            depth_f.CopyFrameDataToArray(tmpDepthImage);

                                            Buffer.BlockCopy(tmpDepthImage, 0, depthImage, 0, depthImage.Length);
                                        }
                                    }
                                }
                                if (selection.infraImage)
                                {
                                    InfraredFrame infra_f = ms_f.InfraredFrameReference.AcquireFrame();
                                    using (infra_f)
                                    {
                                        if (infra_f != null)
                                        {
                                            infra_f.CopyFrameDataToArray(tmpInfraImage);

                                            Buffer.BlockCopy(tmpInfraImage, 0, infraImage, 0, infraImage.Length);
                                        }
                                    }
                                }
                                if (selection.bodyImage)
                                {
                                    BodyIndexFrame body_if = ms_f.BodyIndexFrameReference.AcquireFrame();
                                    using (body_if)
                                    {
                                        if (body_if != null)
                                        {
                                            body_if.CopyFrameDataToArray(bodyImage);
                                        }
                                    }
                                }
                                if (selection.bodyFrame)
                                {
                                    BodyFrame body_f = ms_f.BodyFrameReference.AcquireFrame();
                                    using (body_f)
                                    {
                                        if (body_f != null)
                                        {
                                            body_f.GetAndRefreshBodyData(bodies);

                                            for (int i = 0; i < currentFrame.MaxBodies; i++)
                                            {
                                                if (bodies[i] != null)
                                                {
                                                    if (bodies[i].IsTracked)
                                                    {
                                                        if (!faceFrameSources[i].IsTrackingIdValid)
                                                        {
                                                            faceFrameSources[i].TrackingId = bodies[i].TrackingId;
                                                        }

                                                        bodyFrame[i].isTracked = true;

                                                        if (isBodyTracked[i] == false)
                                                        {
                                                            bodyFrame[i].firstTrack = true;
                                                        }
                                                        else
                                                        {
                                                            bodyFrame[i].firstTrack = false;
                                                        }

                                                        isBodyTracked[i] = true;

                                                        bodyFrame[i].leanLR = bodies[i].Lean.X;
                                                        bodyFrame[i].leanFB = bodies[i].Lean.Y;
                                                        bodyFrame[i].leanConfidence = (int)bodies[i].LeanTrackingState;

                                                        var cameraSP = new CameraSpacePoint();
                                                        var colorSP = new ColorSpacePoint();
                                                        var depthSP = new DepthSpacePoint();

                                                        for (int j = 0; j < currentFrame.MaxJoints; j++)
                                                        {
                                                            bodyFrame[i].skeletonData.jointsRaw[j].x = bodies[i].Joints[(JointType)j].Position.X;
                                                            bodyFrame[i].skeletonData.jointsRaw[j].y = bodies[i].Joints[(JointType)j].Position.Y;
                                                            bodyFrame[i].skeletonData.jointsRaw[j].z = bodies[i].Joints[(JointType)j].Position.Z;
                                                            bodyFrame[i].skeletonData.jointsRaw[j].confidence = (int)bodies[i].Joints[(JointType)j].TrackingState;

                                                            cameraSP.X = bodyFrame[i].skeletonData.jointsRaw[j].x;
                                                            cameraSP.Y = bodyFrame[i].skeletonData.jointsRaw[j].y;
                                                            cameraSP.Z = bodyFrame[i].skeletonData.jointsRaw[j].z;

                                                            colorSP = mapper.MapCameraPointToColorSpace(cameraSP);

                                                            bodyFrame[i].skeletonData.jointsRawColor[j].x = (float)Math.Round(colorSP.X);
                                                            bodyFrame[i].skeletonData.jointsRawColor[j].y = (float)Math.Round(colorSP.Y);
                                                            bodyFrame[i].skeletonData.jointsRawColor[j].z = -1;
                                                            bodyFrame[i].skeletonData.jointsRawColor[j].confidence = bodyFrame[i].skeletonData.jointsRaw[j].confidence;

                                                            depthSP = mapper.MapCameraPointToDepthSpace(cameraSP);

                                                            bodyFrame[i].skeletonData.jointsRawGray[j].x = (float)Math.Round(depthSP.X);
                                                            bodyFrame[i].skeletonData.jointsRawGray[j].y = (float)Math.Round(depthSP.Y);
                                                            bodyFrame[i].skeletonData.jointsRawGray[j].z = -1;
                                                            bodyFrame[i].skeletonData.jointsRawGray[j].confidence = bodyFrame[i].skeletonData.jointsRaw[j].confidence;
                                                        }

                                                        // Tobit Kalman Filtering
                                                        if (bodyFrame[i].firstTrack)
                                                        {
                                                            lastFiltered[i] = bodyFrame[i].skeletonData.jointsRaw;
                                                            for (int j = 0; j < currentFrame.MaxJoints; j++)
                                                            {
                                                                ErrorCovPost[i][j, 0] = 0.01;
                                                                ErrorCovPost[i][j, 1] = 0.01;
                                                                ErrorCovPost[i][j, 2] = 0.01;
                                                            }
                                                        }

                                                        bodyFrame[i].skeletonData.jointsFiltered = TobitKalmanFilter(ref lastFiltered[i], bodyFrame[i].skeletonData.jointsRaw,
                                                                                                                     ref ErrorCovPost[i], currentFrame.MaxJoints);

                                                        for (int j = 0; j < currentFrame.MaxJoints; j++)
                                                        {
                                                            cameraSP.X = bodyFrame[i].skeletonData.jointsFiltered[j].x;
                                                            cameraSP.Y = bodyFrame[i].skeletonData.jointsFiltered[j].y;
                                                            cameraSP.Z = bodyFrame[i].skeletonData.jointsFiltered[j].z;

                                                            colorSP = mapper.MapCameraPointToColorSpace(cameraSP);

                                                            bodyFrame[i].skeletonData.jointsFilteredColor[j].x = (float)Math.Round(colorSP.X);
                                                            bodyFrame[i].skeletonData.jointsFilteredColor[j].y = (float)Math.Round(colorSP.Y);
                                                            bodyFrame[i].skeletonData.jointsFilteredColor[j].z = -1;
                                                            bodyFrame[i].skeletonData.jointsFilteredColor[j].confidence = bodyFrame[i].skeletonData.jointsFiltered[j].confidence;

                                                            depthSP = mapper.MapCameraPointToDepthSpace(cameraSP);

                                                            bodyFrame[i].skeletonData.jointsFilteredGray[j].x = (float)Math.Round(depthSP.X);
                                                            bodyFrame[i].skeletonData.jointsFilteredGray[j].y = (float)Math.Round(depthSP.Y);
                                                            bodyFrame[i].skeletonData.jointsFilteredGray[j].z = -1;
                                                            bodyFrame[i].skeletonData.jointsFilteredGray[j].confidence = bodyFrame[i].skeletonData.jointsFiltered[j].confidence;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        bodyFrame[i].isTracked = false;
                                                        isBodyTracked[i] = false;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                time = DateTime.Now;
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("reader_MultiSourceFrameArrived: " + ex.Message);
                    }
                }

                private void reader_FaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
                {
                    lock (frameLock2)
                    {
                        if (selection.faceBoundingBox)
                        {
                            using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
                            {
                                if (faceFrame != null)
                                {
                                    int index = GetFaceSourceIndex(faceFrame.FaceFrameSource);

                                    if (faceFrame.FaceFrameResult != null)
                                    {
                                        faceFrameResults[index] = faceFrame.FaceFrameResult;

                                        faceBoxColor[index].Bottom = faceFrame.FaceFrameResult.FaceBoundingBoxInColorSpace.Bottom;
                                        faceBoxColor[index].Left = faceFrame.FaceFrameResult.FaceBoundingBoxInColorSpace.Left;
                                        faceBoxColor[index].Right = faceFrame.FaceFrameResult.FaceBoundingBoxInColorSpace.Right;
                                        faceBoxColor[index].Top = faceFrame.FaceFrameResult.FaceBoundingBoxInColorSpace.Top;

                                        faceBoxInfra[index].Bottom = faceFrame.FaceFrameResult.FaceBoundingBoxInInfraredSpace.Bottom;
                                        faceBoxInfra[index].Left = faceFrame.FaceFrameResult.FaceBoundingBoxInInfraredSpace.Left;
                                        faceBoxInfra[index].Right = faceFrame.FaceFrameResult.FaceBoundingBoxInInfraredSpace.Right;
                                        faceBoxInfra[index].Top = faceFrame.FaceFrameResult.FaceBoundingBoxInInfraredSpace.Top;
                                    }
                                }
                            }
                        }
                    }
                }

                #endregion

                /// <summary>Indicates whether the sensor is available.</summary>
                public bool IsSensorAvailable(int msTimeout)
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    while (sensor.IsAvailable == false)
                    {
                        Thread.Sleep(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();
                            sensor.Close();

                            return false;
                        }
                    }

                    sw.Stop();

                    return true;
                }

                /// <summary>Opens the sensor.</summary>
                public void OpenSensor()
                {
                    sensor.Open();
                }

                /// <summary>Closes the sensor.</summary>
                public void CloseSensor()
                {
                    sensor.Close();
                }

                /// <summary>Requests the next frame from the sensor.</summary>
                public KinectFrame GetFrame()
                {
                    // wait for a new frame
                    while (sendTime == readerTime)
                    {
                        Thread.Sleep(sleepTime);
                    }

                    currentFrame.SensorID = sensor.UniqueKinectId;

                    sendTime = readerTime;
                    sendCounter++;

                    lock (frameLock1)
                    {
                        lock (frameLock2)
                        {
                            ScaleData();

                            currentFrame.ColorImage = scaledColor;
                            currentFrame.DepthImage = scaledDepth;
                            currentFrame.InfraImage = scaledInfra;
                            currentFrame.BodyImage = scaledBody;
                            currentFrame.BodyFrame = scaledBodyFrame;
                            currentFrame.FaceBoundingBoxColor = scaledFaceBoxColor;
                            currentFrame.FaceBoundingBoxGray = scaledFaceBoxGray;

                            currentFrame.Timestamp = time;

                            // delete unwanted data 
                            if (!selection.colorImage) { currentFrame.ColorImage = null; }
                            if (!selection.depthImage) { currentFrame.DepthImage = null; }
                            if (!selection.infraImage) { currentFrame.InfraImage = null; }
                            if (!selection.bodyImage) { currentFrame.BodyImage = null; }
                            for (int i = 0; i < currentFrame.MaxBodies; i++)
                            {
                                if (!selection.bodyFrame || !currentFrame.BodyFrame[i].isTracked)
                                {
                                    currentFrame.BodyFrame[i].skeletonData.jointsRaw = null;
                                    currentFrame.BodyFrame[i].skeletonData.jointsRawColor = null;
                                    currentFrame.BodyFrame[i].skeletonData.jointsRawGray = null;
                                    currentFrame.BodyFrame[i].skeletonData.jointsFiltered = null;
                                    currentFrame.BodyFrame[i].skeletonData.jointsFilteredColor = null;
                                    currentFrame.BodyFrame[i].skeletonData.jointsFilteredGray = null;
                                    currentFrame.BodyFrame[i].leanFB = 0;
                                    currentFrame.BodyFrame[i].leanLR = 0;
                                    currentFrame.BodyFrame[i].firstTrack = false;
                                }
                                if (!selection.faceBoundingBox)
                                {
                                    currentFrame.FaceBoundingBoxColor = null;
                                    currentFrame.FaceBoundingBoxGray = null;
                                }
                            }

                            // clear old faces and bodies
                            bodyFrame = NewBody();
                            faceBoxColor = NewFace();
                            faceBoxInfra = NewFace();

                            return currentFrame;
                        }
                    }
                }

                /// <summary>Converts the frame into a byte array.</summary>
                public byte[] FrameToBytes(KinectFrame frame)
                {
                    lock (frameLock1)
                    {
                        lock (frameLock2)
                        {
                            frame = Compress(frame);

                            return Serialization.Serialize(frame);
                        }
                    }
                }

                /// <summary>Converts a byte array into a Kinect frame.</summary>
                public KinectFrame BytesToFrame(byte[] frameBytes)
                {
                    try
                    {
                        KinectFrame frame = Serialization.Deserialize<KinectFrame>(frameBytes);

                        return Decompress(frame);
                    }
                    catch (Exception)
                    {
                        return null;
                    }
                }

                private void ScaleData()
                {
                    scaledColor = FrameOperations.Transform.ScaleColor(scaleColor, colorImage, defaultWidthColor, defaultHeightColor, scalingColor);
                    scaledDepth = FrameOperations.Transform.ScaleGray16(scaleGray, depthImage, defaultWidthGray, defaultHeightGray, scalingGray);
                    scaledInfra = FrameOperations.Transform.ScaleGray16(scaleGray, infraImage, defaultWidthGray, defaultHeightGray, scalingGray);
                    scaledBody = FrameOperations.Transform.ScaleGray(scaleGray, bodyImage, defaultWidthGray, defaultHeightGray, scalingGray);
                    scaledBodyFrame = FrameOperations.Transform.ScaleBodyFrame(bodyFrame, scaleColor, scalingColor, defaultWidthColor, defaultHeightColor,
                                                                                          scaleGray, scalingGray, defaultWidthGray, defaultHeightGray);
                    scaledFaceBoxColor = FrameOperations.Transform.ScaleFaceBox(faceBoxColor, scaleColor, scalingColor, defaultWidthColor, defaultHeightColor);
                    scaledFaceBoxGray = FrameOperations.Transform.ScaleFaceBox(faceBoxInfra, scaleGray, scalingGray, defaultWidthGray, defaultHeightGray);
                }

                private KinectFrame Compress(KinectFrame frame)
                {
                    List<Task> tasks = new List<Task>(4);

                    try
                    {
                        if (frame.ColorImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.ColorImage = FrameOperations.Compress.CompressImage(frame.ColorImage, frame.WidthColor, frame.HeightColor, FrameOperations.ColorTypes.Bgra, compressionColor);
                            }));
                        }
                        if (frame.DepthImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.DepthImage = FrameOperations.Compress.CompressImage(frame.DepthImage, frame.WidthGray, frame.HeightGray, FrameOperations.ColorTypes.Gray16, compressionGray);
                            }));
                        }
                        if (frame.InfraImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.InfraImage = FrameOperations.Compress.CompressImage(frame.InfraImage, frame.WidthGray, frame.HeightGray, FrameOperations.ColorTypes.Gray16, compressionGray);
                            }));
                        }
                        if (frame.BodyImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.BodyImage = FrameOperations.Compress.CompressImage(frame.BodyImage, frame.WidthGray, frame.HeightGray, FrameOperations.ColorTypes.Gray8, -1);
                            }));
                        }

                        foreach (var t in tasks)
                        {
                            t.Wait();
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Compress: " + ex.Message);
                    }

                    return frame;
                }

                private KinectFrame Decompress(KinectFrame frame)
                {
                    List<Task> tasks = new List<Task>(4);

                    try
                    {
                        if (frame.ColorImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.ColorImage = FrameOperations.Decompress.DecompressImage(frame.ColorImage, ref frame.WidthColor, ref frame.HeightColor, FrameOperations.ColorTypes.Bgra);
                            }));
                        }
                        if (frame.DepthImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.DepthImage = FrameOperations.Decompress.DecompressImage(frame.DepthImage, ref frame.WidthGray, ref frame.HeightGray, FrameOperations.ColorTypes.Gray16);
                            }));
                        }
                        if (frame.InfraImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.InfraImage = FrameOperations.Decompress.DecompressImage(frame.InfraImage, ref frame.WidthGray, ref frame.HeightGray, FrameOperations.ColorTypes.Gray16);
                            }));
                        }
                        if (frame.BodyImage != null)
                        {
                            tasks.Add(Task.Run(() =>
                            {
                                frame.BodyImage = FrameOperations.Decompress.DecompressImage(frame.BodyImage, ref frame.WidthGray, ref frame.HeightGray, FrameOperations.ColorTypes.Gray8);
                            }));
                        }

                        foreach (var t in tasks)
                        {
                            t.Wait();
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Decompress: " + ex.Message);
                    }

                    return frame;
                }

                private DataFrames.Kinect.Body[] NewBody()
                {
                    DataFrames.Kinect.Body[] b = new DataFrames.Kinect.Body[currentFrame.MaxBodies];
                    for (int i = 0; i < currentFrame.MaxBodies; i++)
                    {
                        b[i].leanFB = 0;
                        b[i].leanLR = 0;
                        b[i].leanConfidence = 0;
                        b[i].isTracked = false;
                        b[i].firstTrack = false;

                        b[i].skeletonData.jointsRaw = new SkeletonJoint[currentFrame.MaxJoints];
                        b[i].skeletonData.jointsRawColor = new SkeletonJoint[currentFrame.MaxJoints];
                        b[i].skeletonData.jointsRawGray = new SkeletonJoint[currentFrame.MaxJoints];
                        b[i].skeletonData.jointsFiltered = new SkeletonJoint[currentFrame.MaxJoints];
                        b[i].skeletonData.jointsFilteredColor = new SkeletonJoint[currentFrame.MaxJoints];
                        b[i].skeletonData.jointsFilteredGray = new SkeletonJoint[currentFrame.MaxJoints];
                    }

                    return b;
                }

                private FaceBox[] NewFace()
                {
                    FaceBox[] fb = new FaceBox[currentFrame.MaxBodies];
                    for (int i = 0; i < currentFrame.MaxBodies; i++)
                    {
                        fb[i].Bottom = -1;
                        fb[i].Left = -1;
                        fb[i].Right = -1;
                        fb[i].Top = -1;
                    }

                    return fb;
                }

                private int GetFaceSourceIndex(FaceFrameSource faceFrameSource)
                {
                    int index = -1;

                    for (int i = 0; i < currentFrame.MaxBodies; i++)
                    {
                        if (this.faceFrameSources[i] == faceFrameSource)
                        {
                            index = i;
                            break;
                        }
                    }

                    return index;
                }

                private SkeletonJoint[] TobitKalmanFilter(ref SkeletonJoint[] prevFiltered, SkeletonJoint[] currentObservation, ref double[,] Error_Cov_post, int maxJoints)
                {
                    const double maxDisplacementX = 0.34;
                    const double maxDisplacementY = 0.18;
                    const double maxDisplacementZ = 0.34;

                    const float varProcessError = 0.0025f;
                    const float varMeasurementError = 0.01f;

                    SkeletonJoint[] filtered = new SkeletonJoint[maxJoints];

                    double[,] measurement = new double[maxJoints, 3];
                    double[,] post = new double[maxJoints, 3];
                    double[,] prior = new double[maxJoints, 3];
                    double[] Q = new double[3] { varProcessError, varProcessError, varProcessError };
                    double[] Error_Cov_prior = new double[3];
                    double[] Tmax = new double[3];
                    double[] Tmin = new double[3];
                    double[] N_above = new double[3];
                    double[] N_below = new double[3];
                    double[] f_above = new double[3];
                    double[] f_below = new double[3];
                    double[] c_above = new double[3];
                    double[] c_below = new double[3];
                    double[] Pun = new double[3];
                    double[] Lamda = new double[3];
                    double[] E = new double[3];
                    double[] r_new = new double[3];
                    double[] Kalman = new double[3];

                    for (int i = 0; i < maxJoints; i++)
                    {
                        // Data obtained from Kinect
                        measurement[i, 0] = currentObservation[i].x;
                        measurement[i, 1] = currentObservation[i].y;
                        measurement[i, 2] = currentObservation[i].z;

                        // The previous estimated data are the new a priori estimations
                        prior[i, 0] = prevFiltered[i].x;
                        prior[i, 1] = prevFiltered[i].y;
                        prior[i, 2] = prevFiltered[i].z;

                        // The uncensored region limits Tmax and Tmin
                        Tmax[0] = prior[i, 0] + maxDisplacementX;
                        Tmax[1] = prior[i, 1] + maxDisplacementY;
                        Tmax[2] = prior[i, 2] + maxDisplacementZ;

                        Tmin[0] = prior[i, 0] - maxDisplacementX;
                        Tmin[1] = prior[i, 1] - maxDisplacementY;
                        Tmin[2] = prior[i, 2] - maxDisplacementZ;

                        for (int k = 0; k < 3; k++)
                        {
                            // Correct the measurements
                            if (measurement[i, k] > Tmax[k])
                            {
                                measurement[i, k] = Tmax[k];
                            }
                            else if (measurement[i, k] < Tmin[k])
                            {
                                measurement[i, k] = Tmin[k];
                            }

                            // Predict Stage 
                            Error_Cov_prior[k] = Error_Cov_post[i, k] + Q[k];

                            // Tobit Stage
                            N_above[k] = (Tmax[k] - prior[i, k]) / Math.Sqrt(varMeasurementError);
                            N_below[k] = (Tmin[k] - prior[i, k]) / Math.Sqrt(varMeasurementError);

                            f_above[k] = Math.Exp(-Math.Pow(N_above[k], 2) / 2) / Math.Sqrt(2 * Math.PI);
                            f_below[k] = Math.Exp(-Math.Pow(N_below[k], 2) / 2) / Math.Sqrt(2 * Math.PI);

                            c_above[k] = 1 - 0.5 * MMath.Erfc(-N_above[k] / Math.Sqrt(2));
                            c_below[k] = 0.5 * MMath.Erfc(-N_below[k] / Math.Sqrt(2));

                            Pun[k] = 1 - c_above[k] - c_below[k];
                            Lamda[k] = (f_below[k] - f_above[k]) / Pun[k];
                            E[k] = (prior[i, k] + Math.Sqrt(varMeasurementError) * Lamda[k]) * Pun[k] + c_below[k] * Tmin[k] + c_above[k] * Tmax[k];
                            r_new[k] = varMeasurementError * (1 + (N_below[k] * f_below[k] - N_above[k] * f_above[k]) / Pun[k] - Math.Pow(Lamda[k], 2));

                            // Update Stage
                            Kalman[k] = (Error_Cov_prior[k] * Pun[k]) / (Error_Cov_prior[k] * Pun[k] + r_new[k]);
                            Error_Cov_post[i, k] = (1 - Kalman[k] * Pun[k]) * Error_Cov_prior[k];
                            post[i, k] = prior[i, k] + Kalman[k] * (measurement[i, k] - E[k]);
                        }

                        filtered[i].x = (float)post[i, 0];
                        filtered[i].y = (float)post[i, 1];
                        filtered[i].z = (float)post[i, 2];
                        filtered[i].confidence = currentObservation[i].confidence;
                    }

                    prevFiltered = filtered;

                    return filtered;
                }

            }
        }

        namespace Zenith
        {
            public class ZenithSenseLib
            {
                [DllImport("ZenithDLL.dll", CallingConvention = CallingConvention.Cdecl)]
                private static extern void InitializeSensorZenith(string fps, string ip);

                [DllImport("ZenithDLL.dll", CallingConvention = CallingConvention.Cdecl)]
                private static extern bool IsSensorAvailableZenith();

                [DllImport("ZenithDLL.dll", CallingConvention = CallingConvention.Cdecl)]
                private static extern bool OpenSensorZenith();

                [DllImport("ZenithDLL.dll", CallingConvention = CallingConvention.Cdecl)]
                private static extern void CloseSensorZenith();

                [DllImport("ZenithDLL.dll", CallingConvention = CallingConvention.Cdecl)]
                private static extern IntPtr GetFrameZenith();

                private int sleepTime;

                private ZenithFrame currentFrame;
                private int compressionLevel;


                public bool IsSensorAvailable(int msTimeout)
                {
                    try
                    {
                        return _IsSensorAvailable(msTimeout);
                    }
                    catch (Exception)
                    {
                        return false;
                    }
                }

                private bool _IsSensorAvailable(int msTimeout)
                {
                    bool? result = null;
                    object aLock = new object();

                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    Task.Run(() =>
                    {
                        var r = IsSensorAvailableZenith();

                        lock (aLock)
                        {
                            result = r;
                        }
                    });

                    while (result == null)
                    {
                        Thread.Sleep(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();

                            lock (aLock)
                            {
                                result = false;
                            }
                        }
                    }

                    sw.Stop();

                    return (bool)result;
                }

                public void OpenSensor()
                {
                    OpenSensorZenith();
                }

                public void CloseSensor()
                {
                    CloseSensorZenith();
                }

                public void InitializeSensor(string fps, string ip, int compression, int width = 1056, int height = 1056)
                {
                    compressionLevel = compression;
                    currentFrame = new ZenithFrame(width, height);

                    InitializeSensorZenith(fps, ip);
                    sleepTime = (int)(1000.0 / int.Parse(fps));
                }

                public ZenithFrame GetFrame()
                {
                    Thread.Sleep(sleepTime);

                    IntPtr FramePtr = GetFrameZenith();

                    byte[] Frame = new byte[currentFrame.Width * currentFrame.Height * 3];
                    Marshal.Copy(FramePtr, Frame, 0, Frame.Length);

                    currentFrame.ColorData = Frame;
                    currentFrame.Timestamp = DateTime.Now;

                    return currentFrame;
                }

                public byte[] FrameToBytes(ZenithFrame frame)
                {
                    frame = Compress(frame);

                    return Serialization.Serialize(frame);
                }

                public ZenithFrame BytesToFrame(byte[] frameBytes)
                {
                    try
                    {
                        ZenithFrame frame = Serialization.Deserialize<ZenithFrame>(frameBytes);

                        return Decompress(frame);
                    }
                    catch (Exception)
                    {
                        return null;
                    }
                }

                private ZenithFrame Compress(ZenithFrame frame)
                {
                    try
                    {
                        if (frame.ColorData != null)
                        {
                            frame.ColorData = FrameOperations.Compress.CompressImage(frame.ColorData, frame.Width, frame.Height, FrameOperations.ColorTypes.Bgr, compressionLevel);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("ZenithFrame Compress: " + ex.Message);
                    }

                    return frame;
                }

                private ZenithFrame Decompress(ZenithFrame frame)
                {
                    try
                    {
                        if (frame.ColorData != null)
                        {
                            frame.ColorData = FrameOperations.Decompress.DecompressImage(frame.ColorData, ref frame.Width, ref frame.Height, FrameOperations.ColorTypes.Bgr);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("ZenithFrame Decompress: " + ex.Message);
                    }

                    return frame;
                }

            }
        }

        namespace Binary
        {
            public class DataBinarySensors : EventArgs
            {
                public String Data { get; set; }
                public DataBinarySensors(String data)
                {
                    Data = data;
                }
            }

            public class BinarySenseLib
            {
                private const int sleepTime = 45;

                public BinaryFrame currentFrame;

                private readonly object mylock = new object();

                private DateTime readerTime = new DateTime();
                private DateTime sendTime = new DateTime();

                public BinarySenseLib bn_s;
                public event EventHandler<DataBinarySensors> event_status_time;

                public void InitializeSensor()
                {
                    currentFrame = new BinaryFrame();
                    bn_s = new BinarySenseLib();

                    // subscribe to the sensors event and print the content
                    bn_s.event_status_time += (d, s) =>
                    {
                        lock (mylock)
                        {
                            readerTime = DateTime.Now;

                            currentFrame.TimeStamp = readerTime;
                            currentFrame.BinarySensorOutput = s.Data;
                        }
                    };

                    // call the python code to start the sensor acquisition
                    Task.Run(() => bn_s.get_sensors_status_and_time());
                }

                public BinaryFrame GetFrame()
                {
                    while (sendTime == readerTime)
                    {
                        Thread.Sleep(sleepTime);
                    }
                    sendTime = readerTime;

                    lock (mylock)
                    {
                        return currentFrame;
                    }
                }

                public byte[] FrameToBytes(BinaryFrame frame)
                {
                    return Serialization.Serialize(frame);
                }

                public BinaryFrame BytesToFrame(byte[] frameBytes)
                {
                    return Serialization.Deserialize<BinaryFrame>(frameBytes);
                }

                public string IsSensorAvailable(int msTimeout)
                {
                    try
                    {
                        return _IsSensorAvailable(msTimeout);
                    }
                    catch (Exception)
                    {
                        return "0";
                    }
                }

                private string _IsSensorAvailable(int msTimeout)
                {
                    string output = null;
                    object aLock = new object();

                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    Task.Run(() =>
                    {
                        Process p = new Process(); // create process (i.e., the python program)
                        p.StartInfo.FileName = "python.exe";
                        p.StartInfo.RedirectStandardOutput = true;
                        p.StartInfo.UseShellExecute = false; // make sure we can read the output from stdout
                        p.StartInfo.Arguments = "python_scripts\\isRaspberryAvalable.py"; // start the python program with two parameters
                        p.Start(); // start the process (the python program)

                        StreamReader s = p.StandardOutput;
                        var result = s.ReadToEnd();

                        lock (aLock)
                        {
                            output = result.Replace("\r\n", string.Empty);
                        }
                    });

                    while (output == null)
                    {
                        Thread.Sleep(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            lock (aLock)
                            {
                                output = "0";
                            }
                        }
                    }

                    sw.Stop();

                    return output;
                }

                private void get_sensors_status_and_time()
                {
                    Process p = new Process(); // create process (i.e., the python program)
                    p.StartInfo.FileName = "python.exe";
                    p.StartInfo.RedirectStandardOutput = true;
                    p.StartInfo.UseShellExecute = false; // make sure we can read the output from stdout
                    p.StartInfo.Arguments = "python_scripts\\getData_from_raspberry.py"; // start the python program with two parameters

                    //subscribing to output event
                    p.EnableRaisingEvents = true;
                    var local_event_status_time = event_status_time;
                    p.OutputDataReceived += (k, o) =>
                    {
                        //print output event in real time
                        //Console.WriteLine(o.Data);
                        if (local_event_status_time != null)
                        {
                            //send event to interface
                            local_event_status_time(this, new DataBinarySensors(o.Data));
                        }
                    };

                    p.Start(); // start the process (the python program)

                    p.BeginOutputReadLine();

                    p.WaitForExit();
                }

            }
        }

        namespace WSN
        {
            public class WSNSenseLib
            {

                private WSNFrame currentFrame;

                private string WSNLink;
                private int WSNPeriod;


                public WSNSenseLib()
                {
                    currentFrame = new WSNFrame();

                    WSNLink = ""; // enter your IP here
                    WSNPeriod = 15000;
                }

                public void UpdateConnectionLink(string url)
                {
                    WSNLink = url;
                }

                public void UpdateConnectionPeriod(int period)
                {
                    WSNPeriod = period;
                }

                public virtual bool IsSensorAvailable(int msTimeout)
                {
                    bool? result = null;
                    object aLock = new object();

                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    Task.Run(() =>
                    {
                        bool r = false;

                        try
                        {
                            string urlcheck = WSNLink + "/check_sensors";
                            var request = (HttpWebRequest)WebRequest.Create(urlcheck);
                            var response = (HttpWebResponse)request.GetResponse();
                            var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                            if (int.Parse(responseString) == 4)
                            {
                                r = true;
                            }
                        }
                        catch (Exception)
                        {
                            r = false;
                        }

                        lock (aLock)
                        {
                            result = r;
                        }
                    });

                    while (result == null)
                    {
                        Thread.Sleep(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();

                            lock (aLock)
                            {
                                result = false;
                            }
                        }
                    }

                    sw.Stop();

                    return (bool)result;
                }

                public virtual int InitializeSensor()
                {
                    try
                    {
                        string urlget = WSNLink + "/get_data";
                        var request = (HttpWebRequest)WebRequest.Create(urlget);
                        var response = (HttpWebResponse)request.GetResponse();
                        var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                        return int.Parse(responseString);
                    }
                    catch (Exception)
                    {
                        int nocon = -1;
                        return nocon;
                    }
                }

                public virtual int CloseSensor()
                {
                    try
                    {
                        string urlcheck = WSNLink + "/stop_sensors";
                        var request = (HttpWebRequest)WebRequest.Create(urlcheck);
                        var response = (HttpWebResponse)request.GetResponse();
                        var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                        return int.Parse(responseString);
                    }
                    catch (Exception)
                    {
                        int nocon = -1;
                        return nocon;
                    }
                }

                public virtual WSNFrame GetFrame()
                {
                    try
                    {
                        Thread.Sleep(WSNPeriod);

                        string urlget = WSNLink + "/position/xbee/";
                        DateTime actualDateTime = DateTime.Now.IsDaylightSavingTime() ? DateTime.UtcNow.AddHours(2) : DateTime.UtcNow.AddHours(1);
                        DateTime anteriorDateTime = actualDateTime.AddMilliseconds(-WSNPeriod);

                        string timeanterior = anteriorDateTime.ToString("HH:mm:ss");
                        String dateend = actualDateTime.ToString("dd-MM-yyyy");
                        String timeend = actualDateTime.ToString("HH:mm:ss");
                        String timeinit = anteriorDateTime.ToString("HH:mm:ss");
                        String dateinit = anteriorDateTime.ToString("dd-MM-yyyy");

                        Uri myUri = new Uri(urlget + timeinit + "/" + dateinit + "/" + timeend + "/" + dateend + "/");
                        var myHttpWebRequest = (HttpWebRequest)WebRequest.Create(myUri);

                        myHttpWebRequest.ContentType = "application/json; charset=utf-8";
                        var httpResponse = (HttpWebResponse)myHttpWebRequest.GetResponse();
                        using (var streamReader = new StreamReader(httpResponse.GetResponseStream()))
                        {
                            string result = streamReader.ReadToEnd();
                            if (result == "")
                            {
                                result = "no data";
                            }

                            currentFrame.WSNOutput = result;
                        }
                    }
                    catch (Exception ex)
                    {
                        currentFrame.WSNOutput = ex.Message;
                    }

                    return currentFrame;
                }

                public byte[] FrameToBytes(WSNFrame frame)
                {
                    return Serialization.Serialize(frame);
                }

                public WSNFrame BytesToFrame(byte[] frameBytes)
                {
                    return Serialization.Deserialize<WSNFrame>(frameBytes);
                }

            }
        }

        namespace Hexiwear
        {
            public class HexiwearBandSenseLib
            {

                private HexiwearBandFrame currentFrame;

                private string HexiwearLink;
                private int HexiwearPeriod;


                public HexiwearBandSenseLib()
                {
                    currentFrame = new HexiwearBandFrame();

                    HexiwearLink = ""; // enter your IP here
                    HexiwearPeriod = 15000;
                }

                public void UpdateConnectionLink(string url)
                {
                    HexiwearLink = url;
                }

                public void UpdateConnectionPeriod(int period)
                {
                    HexiwearPeriod = period;
                }

                public virtual bool IsSensorAvailable(int msTimeout)
                {
                    bool? result = null;
                    object aLock = new object();

                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    Task.Run(() =>
                    {
                        bool r = false;

                        try
                        {
                            string urlcheck = HexiwearLink + "/check_hexiwear/";
                            var request = (HttpWebRequest)WebRequest.Create(urlcheck);
                            var response = (HttpWebResponse)request.GetResponse();
                            var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                            if (int.Parse(responseString) > 0) // # of bands
                            {
                                r = true;
                            }
                        }
                        catch (Exception)
                        {
                            r = false;
                        }

                        lock (aLock)
                        {
                            result = r;
                        }
                    });

                    while (result == null)
                    {
                        Thread.Sleep(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();

                            lock (aLock)
                            {
                                result = false;
                            }
                        }
                    }

                    sw.Stop();

                    return (bool)result;
                }

                public virtual int InitializeSensor()
                {
                    try
                    {
                        string urlget = HexiwearLink + "/get_data_hexiwear/";
                        var request = (HttpWebRequest)WebRequest.Create(urlget);
                        var response = (HttpWebResponse)request.GetResponse();
                        var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                        return int.Parse(responseString);
                    }
                    catch (Exception)
                    {
                        int nocon = -1;
                        return nocon;
                    }
                }

                public virtual int CloseSensor()
                {
                    try
                    {
                        string urlcheck = HexiwearLink + "/stop_hexiwear/";
                        var request = (HttpWebRequest)WebRequest.Create(urlcheck);
                        var response = (HttpWebResponse)request.GetResponse();
                        var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                        return int.Parse(responseString);
                    }
                    catch (Exception)
                    {
                        int nocon = -1;
                        return nocon;
                    }
                }

                public virtual HexiwearBandFrame GetFrame()
                {
                    try
                    {
                        Thread.Sleep(HexiwearPeriod);

                        Uri urlget = new Uri(HexiwearLink + "/get_database_hexiwear/");
                        DateTime actualDateTime = DateTime.Now.IsDaylightSavingTime() ? DateTime.UtcNow.AddHours(2) : DateTime.UtcNow.AddHours(1);
                        DateTime anteriorDateTime = actualDateTime.AddMilliseconds(-HexiwearPeriod);

                        string timeanterior = anteriorDateTime.ToString("HH:mm:ss");
                        String dateend = actualDateTime.ToString("dd-MM-yyyy");
                        String timeend = actualDateTime.ToString("HH:mm:ss");
                        String timeinit = anteriorDateTime.ToString("HH:mm:ss");
                        String dateinit = anteriorDateTime.ToString("dd-MM-yyyy");

                        Uri myUri = new Uri(urlget + timeinit + "/" + dateinit + "/" + timeend + "/" + dateend + "/");
                        var myHttpWebRequest = (HttpWebRequest)WebRequest.Create(myUri);

                        myHttpWebRequest.ContentType = "application/json; charset=utf-8";
                        var httpResponse = (HttpWebResponse)myHttpWebRequest.GetResponse();
                        using (var streamReader = new StreamReader(httpResponse.GetResponseStream()))
                        {
                            string result = streamReader.ReadToEnd();
                            if (result == "")
                            {
                                result = "no data";
                            }

                            currentFrame.HexiwearBandOutput = result;
                        }
                    }
                    catch (Exception ex)
                    {
                        currentFrame.HexiwearBandOutput = ex.Message;
                    }

                    return currentFrame;
                }

                public byte[] FrameToBytes(HexiwearBandFrame frame)
                {
                    return Serialization.Serialize(frame);
                }

                public HexiwearBandFrame BytesToFrame(byte[] frameBytes)
                {
                    return Serialization.Deserialize<HexiwearBandFrame>(frameBytes);
                }

            }
        }

        namespace UPMBand
        {
            public class UPMBandSenseLib
            {
                private const int sleepTime = 1000;

                public UPMBandFrame currentFrame;

                private BluetoothClient btClient;
                private string btAddress;
                private string btPass;


                public UPMBandSenseLib()
                {
                    currentFrame = new UPMBandFrame();

                    btAddress = ""; // enter the bluetooth address here
                    btPass = ""; // enter your password here
                }

                public void UpdateConnectionParams(string address, string password)
                {
                    btAddress = address;
                    btPass = password;
                }

                public virtual bool IsSensorAvailable(int msTimeout)
                {
                    bool? result = null;
                    object aLock = new object();

                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    Task.Run(() =>
                    {
                        bool r = false;

                        try
                        {
                            InTheHand.Net.BluetoothAddress addr = InTheHand.Net.BluetoothAddress.Parse(btAddress);
                            var bluetoothClient = new BluetoothClient();
                            var devices = bluetoothClient.DiscoverDevices();

                            foreach (var device in devices)
                            {
                                var blueToothInfo = string.Format("- DeviceName: {0}{1}  Connected: {2}{1}  Address: {3}{1}  Last seen: {4}{1}  Last used: {5}{1}", device.DeviceName, Environment.NewLine, device.Connected, device.DeviceAddress, device.LastSeen, device.LastUsed);
                                blueToothInfo += string.Format("  Class of device{0}   Device: {1}{0}   Major Device: {2}{0}   Service: {3}", Environment.NewLine, device.ClassOfDevice.Device, device.ClassOfDevice.MajorDevice, device.ClassOfDevice.Service);

                                if (device.DeviceAddress == addr)
                                {
                                    r = true;
                                }
                            }
                        }
                        catch (Exception)
                        {
                            r = false;
                        }

                        lock (aLock)
                        {
                            result = r;
                        }
                    });

                    while (result == null)
                    {
                        Thread.Sleep(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();

                            lock (aLock)
                            {
                                result = false;
                            }
                        }
                    }

                    sw.Stop();

                    return (bool)result;
                }

                public virtual void CloseSensor()
                {
                    btClient.Close();
                }


                public virtual bool InitializeSensor()
                {
                    var bluetoothClient = new BluetoothClient();
                    InTheHand.Net.BluetoothAddress addr = InTheHand.Net.BluetoothAddress.Parse(btAddress);
                    bool paired = InTheHand.Net.Bluetooth.BluetoothSecurity.PairRequest(addr, btPass);
                    InTheHand.Net.Sockets.BluetoothDeviceInfo device = new InTheHand.Net.Sockets.BluetoothDeviceInfo(addr);

                    if (device.Authenticated)
                    {
                        bluetoothClient.SetPin(btPass);
                        bluetoothClient.BeginConnect(device.DeviceAddress, InTheHand.Net.Bluetooth.BluetoothService.SerialPort, new AsyncCallback(InitializeSensor), device);

                        int milliseconds = 1000;
                        Thread.Sleep(milliseconds);

                        btClient = bluetoothClient;
                        return btClient.Connected;
                    }
                    else
                    {
                        btClient = bluetoothClient;
                        return btClient.Connected;
                    }
                }

                private void InitializeSensor(IAsyncResult result)
                {
                    if (result.IsCompleted)
                    {

                    }
                }

                public virtual UPMBandFrame GetFrame()
                {
                    try
                    {
                        Thread.Sleep(sleepTime);

                        var stream = btClient.GetStream();
                        string content = btAddress;
                        var buffer = System.Text.Encoding.UTF8.GetBytes(content);
                        stream.Write(buffer, 0, buffer.Length);
                        stream.Flush();

                        UPMBandFrame frame = new UPMBandFrame();
                        if (stream.CanRead)
                        {
                            byte[] myReadBuffer = new byte[1024];
                            System.Text.StringBuilder myCompleteMessage = new System.Text.StringBuilder();
                            int numberOfBytesRead = 0;
                            do
                            {
                                numberOfBytesRead = stream.Read(myReadBuffer, 0, myReadBuffer.Length);
                                myCompleteMessage.AppendFormat("{0}", System.Text.Encoding.ASCII.GetString(myReadBuffer, 0, numberOfBytesRead));
                            }
                            while (stream.DataAvailable);
                            char delimiter = '&';
                            String[] substrings = myCompleteMessage.ToString().Split(delimiter);
                            int items = substrings.Length;
                            DateTime actualDateTime = DateTime.Now;
                            TimeSpan t = DateTime.UtcNow - new DateTime(1970, 1, 1);
                            int secondsSinceEpoch = (int)t.TotalSeconds;

                            foreach (string sub in substrings)
                            {
                                frame.UPMBandOutput += sub + '#' + secondsSinceEpoch.ToString() + "#&";
                                secondsSinceEpoch = secondsSinceEpoch - 1;
                            }

                            return frame;
                        }
                        else
                        {
                            return null;
                        }
                    }
                    catch (Exception)
                    {
                        return null;
                    }
                }

                public byte[] FrameToBytes(UPMBandFrame frame)
                {
                    return Serialization.Serialize(frame);
                }

                public UPMBandFrame BytesToFrame(byte[] frameBytes)
                {
                    return Serialization.Deserialize<UPMBandFrame>(frameBytes);
                }

            }
        }

    }

    namespace Utils
    {
        /// <summary>The supported sensors.</summary>
        public enum SensorType
        {
            Kinect = 0,
            Zenith = 1,
            MSBand = 2,
            HexiwearBand = 3,
            UPMBand = 4,
            Binary = 5,
            WSN = 6
        }

        /// <summary>Provides sensor-related functions.</summary>
        public static class SensorUtils
        {
            /// <summary>Returns the number of supported sensors.</summary>
            public static int EnumSensors()
            {
                return Enum.GetNames(typeof(SensorType)).Length;
            }

            /// <summary>Returns a dictionary with the supported sensors as keys and the given integer (default = 0) as their value.</summary>
            public static Dictionary<SensorType, int> SensorDictionary(int value = 0)
            {
                var d = new Dictionary<SensorType, int>(EnumSensors());
                for (int i = 0; i < EnumSensors(); i++)
                {
                    d.Add((SensorType)i, value);
                }

                return d;
            }

            /// <summary>Returns the total number of sensors in the dictionary.</summary>
            public static int SensorDictionaryCount(Dictionary<SensorType, int> dict)
            {
                int count = 0;
                foreach (var d in dict.Values)
                {
                    count += d;
                }

                return count;
            }

        } // end class

        /// <summary>Provides methods for Serializing, Deep copying and Packaging of arbitrary objects.</summary>
        public static class Serialization
        {

            /// <summary>Serializes the given object into a byte array using a binary format.</summary>
            public static byte[] Serialize(object obj)
            {
                byte[] result;

                BinaryFormatter serializer = new BinaryFormatter();
                using (MemoryStream memStream = new MemoryStream())
                {
                    serializer.Serialize(memStream, obj);
                    memStream.Position = 0;
                    result = new byte[memStream.Length];
                    memStream.Read(result, 0, (int)memStream.Length);

                    return result;
                }
            }

            /// <summary>De-serializes the given object into type T using a binary format.</summary>
            public static T Deserialize<T>(byte[] obj)
            {
                object result;

                BinaryFormatter deserializer = new BinaryFormatter();
                using (MemoryStream memStream = new MemoryStream(obj))
                {
                    memStream.Position = 0;
                    result = deserializer.Deserialize(memStream);

                    return (T)result;
                }
            }

            /// <summary>Serializes the given object into a byte array using an XML format.</summary>
            public static byte[] SerializeXML(object obj)
            {
                byte[] result;

                DataContractSerializer serializer = new DataContractSerializer(obj.GetType());
                using (MemoryStream memStream = new MemoryStream())
                {
                    serializer.WriteObject(memStream, obj);
                    memStream.Position = 0;
                    result = new byte[memStream.Length];
                    memStream.Read(result, 0, (int)memStream.Length);

                    return result;
                }
            }

            /// <summary>De-serializes the given object into type T using an XML format.</summary>
            public static T DeserializeXML<T>(byte[] obj)
            {
                object result;

                DataContractSerializer deserializer = new DataContractSerializer(typeof(T));
                using (MemoryStream memStream = new MemoryStream(obj))
                {
                    memStream.Position = 0;
                    result = deserializer.ReadObject(memStream);

                    return (T)result;
                }
            }

            /// <summary>Returns a deep copy of the given object</summary>
            public static T DeepCopy<T>(T obj)
            {
                using (var ms = new MemoryStream())
                {
                    var formatter = new BinaryFormatter();
                    formatter.Serialize(ms, obj);
                    ms.Position = 0;

                    return (T)formatter.Deserialize(ms);
                }
            }

            /// <summary>Produces a byte array containing the message length and the actual message</summary>
            public static byte[] PackageMessage(byte[] msg)
            {
                byte[] result = new byte[4 + msg.Length];

                Array.Copy(BitConverter.GetBytes(msg.Length), result, 4);
                Array.Copy(msg, 0, result, 4, msg.Length);

                return result;
            }


        } // end class


        /// <summary>Provides methods regarding network connectivity.</summary>
        public static class ConnectionInfo
        {

            /// <summary>Contains the default communication ports of the Sensors Client and the Server.</summary>
            public enum DefaultPorts
            {
                clientListenerMSBand = 1111,// enter your port number here
                clientListener = 2222,      // enter your port number here
                serverListener = 3333       // enter your port number here
            }

            /// <summary>Returns a list of the communication ports in use.</summary>
            public static List<int> GetConnections()
            {
                List<int> usedPorts = new List<int>();

                IPGlobalProperties IPGP = IPGlobalProperties.GetIPGlobalProperties();
                TcpConnectionInformation[] infoConn = IPGP.GetActiveTcpConnections();
                foreach (var conn in infoConn)
                {
                    usedPorts.Add(conn.LocalEndPoint.Port);
                }

                IPEndPoint[] infoLstn = IPGP.GetActiveTcpListeners();
                foreach (var conn in infoLstn)
                {
                    usedPorts.Add(conn.Port);
                }

                return usedPorts;
            }

            /// <summary>Returns an array of free communication ports.</summary>
            public static int[] GetRandomPort(int numOfPorts, int start = 1024, int end = 49151)
            {
                List<int> usedPorts = GetConnections();
                Random rng = new Random();

                int[] result = new int[numOfPorts];
                int count = 0;
                while (count < numOfPorts)
                {
                    int rnd = rng.Next(start, end + 1);
                    while (usedPorts.Contains(rnd))
                    {
                        rnd = rng.Next(start, end + 1);
                    }
                    result[count] = rnd;
                    count++;
                }

                return result;
            }

        } // end class


        /// <summary>Provides methods for number and region validation.</summary>
        public static class Validation
        {

            /// <summary>Checks that the given number is finite. If LBound is less than UBound, it also checks that the number is within the provided limits.</summary>
            public static bool ValidateNumber(double x, double LBound = 0.0, double UBound = 0.0)
            {
                if (LBound < UBound)
                {
                    return !double.IsNaN(x) && !double.IsInfinity(x) && x >= LBound && x <= UBound;
                }
                else // no bound validation
                {
                    return !double.IsNaN(x) && !double.IsInfinity(x);
                }
            }

            /// <summary>Checks that the given number is finite. If LBound is less than UBound, it also checks that the number is within the provided limits.</summary>
            public static bool ValidateNumber(float x, float LBound = 0.0f, float UBound = 0.0f)
            {
                if (LBound < UBound)
                {
                    return !float.IsNaN(x) && !float.IsInfinity(x) && x >= LBound && x <= UBound;
                }
                else // no bound validation
                {
                    return !float.IsNaN(x) && !float.IsInfinity(x);
                }
            }

            /// <summary>Checks that the given number is less than the max value an integer can hold and greater than min value.
            /// If LBound is less than UBound, it also checks that the number is within the provided limits.</summary>
            public static bool ValidateNumber(int x, int LBound = 0, int UBound = 0)
            {
                if (LBound < UBound)
                {
                    return x < int.MaxValue && x > int.MinValue && x >= LBound && x <= UBound;
                }
                else // no bound validation
                {
                    return x < int.MaxValue && x > int.MinValue;
                }

            }

            /// <summary>Checks that the given region is within the given bounds and larger than a single point.</summary>
            public static bool ValidateRegion(Rectangle region, Rectangle bounds)
            {
                return region.Left >= bounds.Left && region.Right < bounds.Right && region.Top >= bounds.Top && region.Bottom < bounds.Bottom && region.Width > 0 && region.Height > 0 &&
                       region.Left < region.Right && region.Top < region.Bottom;
            }

        } // end class


        /// <summary>Provides methods for data storage and retrieval.</summary>
        public static class DataIO
        {

            /// <summary>Provides methods for writing data to a file.</summary>
            public class FileWriter
            {
                private readonly object myLock;
                private BinaryWriter writerBinary;
                private StreamWriter writerText;

                /// <summary>The number of errors during writing.</summary>
                public int ErrorCounter { get; private set; }


                /// <summary>Creates or appends to the specified file.</summary>
                public FileWriter(string filename, bool binaryData)
                {
                    this.myLock = new object();
                    this.ErrorCounter = 0;

                    if (binaryData)
                    {
                        this.writerBinary = new BinaryWriter(File.Open(filename, FileMode.Append, FileAccess.Write));
                    }
                    else
                    {
                        this.writerText = new StreamWriter(filename, true, System.Text.Encoding.UTF8);
                    }
                }

                /// <summary>Creates or appends to the specified file.</summary>
                public FileWriter(string filename, SensorType sensorType, string sensorID, bool binaryData)
                {
                    this.myLock = new object();
                    this.ErrorCounter = 0;

                    if (sensorID != "")
                    {
                        sensorID = " " + sensorID;
                    }

                    if (binaryData)
                    {
                        this.writerBinary = new BinaryWriter(
                            File.Open(filename + Path.AltDirectorySeparatorChar + sensorType.ToString() + sensorID + ".txt", FileMode.Append, FileAccess.Write));
                    }
                    else
                    {
                        this.writerText =
                            new StreamWriter(filename + Path.AltDirectorySeparatorChar + sensorType.ToString() + sensorID + ".txt", true, System.Text.Encoding.UTF8);
                    }
                }

                /// <summary>Writes the given binary data to the file and returns true if the operation completed successfully or false if an error occurred.</summary>
                public bool WriteData(byte[] data)
                {
                    return WriteBinaryData(data);
                }

                /// <summary>Writes the given text to the file and returns true if the operation completed successfully or false if an error occurred.</summary>
                public bool WriteData(string data)
                {
                    return WriteTextData(data);
                }

                /// <summary>Writes the new line character to the file (when this instance is initialized with binaryData = false)
                /// and returns true if the operation completed successfully or false if an error occurred.</summary>
                public bool WriteNewLine()
                {
                    return WriteTextData("\r\n");
                }

                private bool WriteBinaryData(byte[] data)
                {
                    lock (myLock)
                    {
                        try
                        {
                            writerBinary.Write(data.Length);
                            writerBinary.Write(data);

                            return true;
                        }
                        catch (Exception)
                        {
                            this.ErrorCounter += 1;

                            return false;
                        }
                    }
                }

                private bool WriteTextData(string data)
                {
                    lock (myLock)
                    {
                        try
                        {
                            writerText.Write(data);

                            return true;
                        }
                        catch (Exception)
                        {
                            this.ErrorCounter += 1;

                            return false;
                        }
                    }
                }

                /// <summary>Resets the error counter.</summary>
                public void ResetErrors()
                {
                    this.ErrorCounter = 0;
                }

                /// <summary>Closes the file and releases resources.</summary>
                public void Close()
                {
                    writerBinary?.Close();
                    writerText?.Close();
                }

            } // end class

            /// <summary>Provides methods for reading binary data from a file.</summary>
            public class FileReader
            {
                private BinaryReader reader;
                private int len;
                private long total;

                private byte[] buffer;
                private int bufferSize;

                private readonly int frameLBound;
                private readonly int frameUBound;

                private readonly byte[] eof;

                private readonly int frameSignatureHeader;
                private readonly byte[] frameSignature;

                /// <summary>Marks the end of the file.</summary>
                public byte[] EoF
                {
                    get { return eof; }
                }


                /// <summary>Opens the given file, sets the read buffer size (default 25MB) and sets the frame length bounds used for validation 
                /// (default values are 0, meaning no validation).</summary>
                public FileReader(string filename, int bufferSize = 25000000, int LBound = 0, int UBound = 0)
                {
                    this.eof = new byte[] { 84, 104, 101, 32, 101, 110, 100 };

                    this.frameSignatureHeader = 22;
                    this.frameSignature = new byte[] { 63, 83, 101, 110, 115, 101, 76, 105, 98 }; // text: ?SenseLib

                    this.reader = new BinaryReader(File.OpenRead(filename)); //System.Text.Encoding.ASCII);
                    this.len = 0;
                    this.total = 0;

                    this.bufferSize = 25000000;
                    if (Validation.ValidateNumber(bufferSize, frameSignature.Length, int.MaxValue - 1))
                    {
                        this.bufferSize = bufferSize;
                    }

                    this.frameLBound = 0;
                    if (Validation.ValidateNumber(LBound, 0, int.MaxValue - 1))
                    {
                        this.frameLBound = LBound;
                    }

                    this.frameUBound = 0;
                    if (Validation.ValidateNumber(UBound, 0, int.MaxValue - 1))
                    {
                        this.frameUBound = UBound;
                    }
                }

                /// <summary>Reads the given file until the end and returns the number of frames found.
                /// If the frame length exceeded the bounds or an error occurred, the return value is -1.</summary>
                public long EnumFrames()
                {
                    long holdPosition = reader.BaseStream.Position;

                    try
                    {
                        while (reader.BaseStream.Position < reader.BaseStream.Length)
                        {
                            len = reader.ReadInt32();
                            if (frameLBound < frameUBound && (len < frameLBound || len > frameUBound))
                            {
                                reader.BaseStream.Position = holdPosition;
                                return -1;
                            }

                            reader.BaseStream.Position += len;

                            total++;
                        }

                        reader.BaseStream.Position = holdPosition;

                        return total;
                    }
                    catch (Exception)
                    {
                        reader.BaseStream.Position = holdPosition;
                        return -1;
                    }
                }

                /// <summary>Reads and returns the next frame from the file, or EoF if the end has been reached.
                /// If the frame length exceeded the bounds or an error occurred, the return value is null.</summary>
                public byte[] ReadFrame()
                {
                    long holdPosition = reader.BaseStream.Position;

                    try
                    {
                        if (reader.BaseStream.Position < reader.BaseStream.Length)
                        {
                            len = reader.ReadInt32();
                            if (frameLBound < frameUBound && (len < frameLBound || len > frameUBound))
                            {
                                reader.BaseStream.Position = holdPosition;
                                return null;
                            }

                            return reader.ReadBytes(len);
                        }
                        else
                        {
                            return eof;
                        }
                    }
                    catch (Exception)
                    {
                        reader.BaseStream.Position = holdPosition;

                        return null;
                    }
                }

                /// <summary>Reads and returns the next frame from the file using signature matching, or EoF if the end has been reached.
                /// If the frame length exceeded the bounds or an error occurred, the return value is null.</summary>
                public byte[] TryReadFrame()
                {
                    try
                    {
                        while (reader.BaseStream.Position + frameSignature.Length < reader.BaseStream.Length)
                        {
                            buffer = reader.ReadBytes(bufferSize);

                            if (buffer.Length < frameSignature.Length)
                            {
                                return eof;
                            }

                            for (int i = 0; i < buffer.Length - frameSignature.Length; i++)
                            {
                                int pos = -1;

                                for (int j = 0; j < frameSignature.Length; j++)
                                {
                                    if (buffer[i + j] != frameSignature[j])
                                    {
                                        break;
                                    }
                                    pos = j;
                                }

                                if (pos == frameSignature.Length - 1)
                                {
                                    reader.BaseStream.Position = reader.BaseStream.Position - buffer.Length + i - frameSignatureHeader - 4;

                                    len = reader.ReadInt32();
                                    if (frameLBound < frameUBound && (len < frameLBound || len > frameUBound))
                                    {
                                        return null;
                                    }

                                    return reader.ReadBytes(len);
                                }
                            }

                            // create overlap with previous buffer
                            reader.BaseStream.Position -= frameSignature.Length;
                        }

                        return eof;
                    }
                    catch (Exception)
                    {
                        return null;
                    }
                }

                /// <summary>Closes the file and releases resources.</summary>
                public void Close()
                {
                    buffer = null;
                    reader.Close();
                }

            } // end class

            /// <summary>Provides methods for writing data to a MongoDB database.</summary>
            public class DBWriter
            {
                protected static IMongoClient client;
                protected static IMongoDatabase database;

                private IMongoCollection<BsonDocument> tableKinect;
                private IMongoCollection<BsonDocument> tableZenith;
                private IMongoCollection<BsonDocument> tableMSBand;
                private IMongoCollection<BsonDocument> tableHexiwearBand;
                private IMongoCollection<BsonDocument> tableUPMBand;
                private IMongoCollection<BsonDocument> tableBinary;
                private IMongoCollection<BsonDocument> tableWSN;
                private IMongoCollection<BsonDocument> tableBandPersonIDs;

                private readonly object myLock;

                /// <summary>The number of errors during writing.</summary>
                public int ErrorCounter { get; private set; }


                /// <summary>Connects to the given MongoDB database.</summary>
                public DBWriter(string db)
                {
                    client = new MongoClient();
                    database = client.GetDatabase(db);

                    tableKinect = database.GetCollection<BsonDocument>("Kinect");
                    tableZenith = database.GetCollection<BsonDocument>("Zenith");
                    tableMSBand = database.GetCollection<BsonDocument>("MSBand");
                    tableHexiwearBand = database.GetCollection<BsonDocument>("HexiwearBand");
                    tableUPMBand = database.GetCollection<BsonDocument>("UPMBand");
                    tableBinary = database.GetCollection<BsonDocument>("Binary");
                    tableWSN = database.GetCollection<BsonDocument>("WSN");
                    tableBandPersonIDs = database.GetCollection<BsonDocument>("BandPersonIDs");

                    myLock = new object();

                    ErrorCounter = 0;
                }

                /// <summary>Checks the connection status.</summary>
                public async Task<bool> IsConnected(int msTimeout = 5000)
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    while (client.Cluster.Description.State != MongoDB.Driver.Core.Clusters.ClusterState.Connected)
                    {
                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();

                            return false;
                        }

                        await Task.Delay(msTimeout / 4);
                    }

                    sw.Stop();

                    return true;
                }

                /// <summary>Writes the given binary data to the database.</summary>
                public async Task<bool> WriteBinaryData(SensorType sensorType, DateTime id, string session, string clientName, byte[] data)
                {
                    try
                    {
                        var document = new BsonDocument();

                        document.Add("_id", BsonValue.Create(id));
                        document.Add("Session", BsonValue.Create(session));
                        document.Add("Client", BsonValue.Create(clientName));
                        document.Add("Data", BsonValue.Create(data));

                        if (sensorType == SensorType.Kinect)
                        {
                            await tableKinect.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.Zenith)
                        {
                            await tableZenith.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.MSBand)
                        {
                            await tableMSBand.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.HexiwearBand)
                        {
                            await tableHexiwearBand.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.UPMBand)
                        {
                            await tableUPMBand.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.Binary)
                        {
                            await tableBinary.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.WSN)
                        {
                            await tableWSN.InsertOneAsync(document);
                        }
                        else
                        {
                            throw new Exception("This sensor is not supported yet.");
                        }

                        return true;
                    }
                    catch (Exception)
                    {
                        lock (myLock)
                        {
                            ErrorCounter += 1;
                        }

                        return false;
                    }
                }

                /// <summary>Writes the given sensor frame to the database.</summary>
                public async Task<bool> WriteData(SensorType sensorType, DateTime id, string session, string clientName, object data)
                {
                    try
                    {
                        var document = new BsonDocument();

                        document.Add("_id", BsonValue.Create(id));
                        document.Add("Session", BsonValue.Create(session));
                        document.Add("Client", BsonValue.Create(clientName));

                        if (sensorType == SensorType.Kinect)
                        {
                            var frameKinect = (KinectFrame)data;

                            document.Add("SensorID", BsonValue.Create(frameKinect.SensorID));

                            document.Add("ColorImage", BsonValue.Create(frameKinect.ColorImage));
                            document.Add("DepthImage", BsonValue.Create(frameKinect.DepthImage));
                            document.Add("InfraImage", BsonValue.Create(frameKinect.InfraImage));
                            document.Add("BodyImage", BsonValue.Create(frameKinect.BodyImage));

                            var docBodies = new BsonArray();
                            for (int i = 0; i < frameKinect.MaxBodies; i++)
                            {
                                var docBodyFrame = new BsonDocument();
                                docBodyFrame.Add("id", BsonValue.Create(i));
                                docBodyFrame.Add("leanFB", BsonValue.Create(frameKinect.BodyFrame[i].leanFB));
                                docBodyFrame.Add("leanLR", BsonValue.Create(frameKinect.BodyFrame[i].leanLR));
                                docBodyFrame.Add("leanConfidence", BsonValue.Create(frameKinect.BodyFrame[i].leanConfidence));
                                docBodyFrame.Add("isTracked", BsonValue.Create(frameKinect.BodyFrame[i].isTracked));
                                docBodyFrame.Add("firstTrack", BsonValue.Create(frameKinect.BodyFrame[i].firstTrack));

                                var docSkeleton = new BsonDocument();

                                // raw
                                if (frameKinect.BodyFrame[i].skeletonData.jointsRaw == null)
                                {
                                    docSkeleton.Add("raw", BsonValue.Create(null));
                                }
                                else
                                {
                                    var docSkeletonRawJoints = new BsonArray();
                                    for (int j = 0; j < frameKinect.MaxJoints; j++)
                                    {
                                        var docSkeletonRawJointValues = new BsonDocument();
                                        docSkeletonRawJointValues.Add("id", BsonValue.Create(j));
                                        docSkeletonRawJointValues.Add("x", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRaw[j].x));
                                        docSkeletonRawJointValues.Add("y", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRaw[j].y));
                                        docSkeletonRawJointValues.Add("z", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRaw[j].z));
                                        docSkeletonRawJointValues.Add("confidence", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRaw[j].confidence));

                                        docSkeletonRawJoints.Add(docSkeletonRawJointValues);
                                    }
                                    docSkeleton.Add("raw", docSkeletonRawJoints);
                                }

                                // raw Color
                                if (frameKinect.BodyFrame[i].skeletonData.jointsRawColor == null)
                                {
                                    docSkeleton.Add("rawColor", BsonValue.Create(null));
                                }
                                else
                                {
                                    var docSkeletonRawColorJoints = new BsonArray();
                                    for (int j = 0; j < frameKinect.MaxJoints; j++)
                                    {
                                        var docSkeletonRawColorJointValues = new BsonDocument();
                                        docSkeletonRawColorJointValues.Add("id", BsonValue.Create(j));
                                        docSkeletonRawColorJointValues.Add("x", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawColor[j].x));
                                        docSkeletonRawColorJointValues.Add("y", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawColor[j].y));
                                        docSkeletonRawColorJointValues.Add("z", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawColor[j].z));
                                        docSkeletonRawColorJointValues.Add("confidence", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawColor[j].confidence));

                                        docSkeletonRawColorJoints.Add(docSkeletonRawColorJointValues);
                                    }
                                    docSkeleton.Add("rawColor", docSkeletonRawColorJoints);
                                }

                                // raw Gray
                                if (frameKinect.BodyFrame[i].skeletonData.jointsRawGray == null)
                                {
                                    docSkeleton.Add("rawGray", BsonValue.Create(null));
                                }
                                else
                                {
                                    var docSkeletonRawDepthJoints = new BsonArray();
                                    for (int j = 0; j < frameKinect.MaxJoints; j++)
                                    {
                                        var docSkeletonRawDepthJointValues = new BsonDocument();
                                        docSkeletonRawDepthJointValues.Add("id", BsonValue.Create(j));
                                        docSkeletonRawDepthJointValues.Add("x", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawGray[j].x));
                                        docSkeletonRawDepthJointValues.Add("y", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawGray[j].y));
                                        docSkeletonRawDepthJointValues.Add("z", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawGray[j].z));
                                        docSkeletonRawDepthJointValues.Add("confidence", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsRawGray[j].confidence));

                                        docSkeletonRawDepthJoints.Add(docSkeletonRawDepthJointValues);
                                    }
                                    docSkeleton.Add("rawGray", docSkeletonRawDepthJoints);
                                }

                                // filtered
                                if (frameKinect.BodyFrame[i].skeletonData.jointsFiltered == null)
                                {
                                    docSkeleton.Add("filtered", BsonValue.Create(null));
                                }
                                else
                                {
                                    var docSkeletonFilteredJoints = new BsonArray();
                                    for (int j = 0; j < frameKinect.MaxJoints; j++)
                                    {
                                        var docSkeletonFilteredJointValues = new BsonDocument();
                                        docSkeletonFilteredJointValues.Add("id", BsonValue.Create(j));
                                        docSkeletonFilteredJointValues.Add("x", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFiltered[j].x));
                                        docSkeletonFilteredJointValues.Add("y", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFiltered[j].y));
                                        docSkeletonFilteredJointValues.Add("z", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFiltered[j].z));
                                        docSkeletonFilteredJointValues.Add("confidence", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFiltered[j].confidence));

                                        docSkeletonFilteredJoints.Add(docSkeletonFilteredJointValues);
                                    }
                                    docSkeleton.Add("filtered", docSkeletonFilteredJoints);
                                }

                                // filtered Color
                                if (frameKinect.BodyFrame[i].skeletonData.jointsFilteredColor == null)
                                {
                                    docSkeleton.Add("filteredColor", BsonValue.Create(null));
                                }
                                else
                                {
                                    var docSkeletonFilteredColorJoints = new BsonArray();
                                    for (int j = 0; j < frameKinect.MaxJoints; j++)
                                    {
                                        var docSkeletonFilteredColorJointValues = new BsonDocument();
                                        docSkeletonFilteredColorJointValues.Add("id", BsonValue.Create(j));
                                        docSkeletonFilteredColorJointValues.Add("x", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredColor[j].x));
                                        docSkeletonFilteredColorJointValues.Add("y", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredColor[j].y));
                                        docSkeletonFilteredColorJointValues.Add("z", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredColor[j].z));
                                        docSkeletonFilteredColorJointValues.Add("confidence", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredColor[j].confidence));

                                        docSkeletonFilteredColorJoints.Add(docSkeletonFilteredColorJointValues);
                                    }
                                    docSkeleton.Add("filteredColor", docSkeletonFilteredColorJoints);
                                }

                                // filtered Depth
                                if (frameKinect.BodyFrame[i].skeletonData.jointsFilteredGray == null)
                                {
                                    docSkeleton.Add("filteredGray", BsonValue.Create(null));
                                }
                                else
                                {
                                    var docSkeletonFilteredDepthJoints = new BsonArray();
                                    for (int j = 0; j < frameKinect.MaxJoints; j++)
                                    {
                                        var docSkeletonFilteredDepthJointValues = new BsonDocument();
                                        docSkeletonFilteredDepthJointValues.Add("id", BsonValue.Create(j));
                                        docSkeletonFilteredDepthJointValues.Add("x", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredGray[j].x));
                                        docSkeletonFilteredDepthJointValues.Add("y", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredGray[j].y));
                                        docSkeletonFilteredDepthJointValues.Add("z", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredGray[j].z));
                                        docSkeletonFilteredDepthJointValues.Add("confidence", BsonValue.Create(frameKinect.BodyFrame[i].skeletonData.jointsFilteredGray[j].confidence));

                                        docSkeletonFilteredDepthJoints.Add(docSkeletonFilteredDepthJointValues);
                                    }
                                    docSkeleton.Add("filteredGray", docSkeletonFilteredDepthJoints);
                                }

                                docBodyFrame.Add("skeleton", docSkeleton);
                                docBodies.Add(docBodyFrame);
                            }
                            document.Add("BodyFrame", docBodies);

                            // face bounding box
                            var docFaceColor = new BsonArray();
                            var docFaceGray = new BsonArray();
                            for (int i = 0; i < frameKinect.MaxBodies; i++)
                            {
                                var docBBColor = new BsonDocument();
                                if (frameKinect.FaceBoundingBoxColor == null)
                                {
                                    docFaceColor = null;
                                }
                                else
                                {
                                    docBBColor.Add("id", BsonValue.Create(i));
                                    docBBColor.Add("Bottom", BsonValue.Create(frameKinect.FaceBoundingBoxColor[i].Bottom));
                                    docBBColor.Add("Left", BsonValue.Create(frameKinect.FaceBoundingBoxColor[i].Left));
                                    docBBColor.Add("Right", BsonValue.Create(frameKinect.FaceBoundingBoxColor[i].Right));
                                    docBBColor.Add("Top", BsonValue.Create(frameKinect.FaceBoundingBoxColor[i].Top));

                                    docFaceColor.Add(docBBColor);
                                }

                                var docBBGray = new BsonDocument();
                                if (frameKinect.FaceBoundingBoxGray == null)
                                {
                                    docFaceGray = null;
                                }
                                else
                                {
                                    docBBGray.Add("id", BsonValue.Create(i));
                                    docBBGray.Add("Bottom", BsonValue.Create(frameKinect.FaceBoundingBoxGray[i].Bottom));
                                    docBBGray.Add("Left", BsonValue.Create(frameKinect.FaceBoundingBoxGray[i].Left));
                                    docBBGray.Add("Right", BsonValue.Create(frameKinect.FaceBoundingBoxGray[i].Right));
                                    docBBGray.Add("Top", BsonValue.Create(frameKinect.FaceBoundingBoxGray[i].Top));

                                    docFaceGray.Add(docBBGray);
                                }
                            }

                            if (docFaceColor == null)
                            {
                                document.Add("FaceBoundingBoxColor", BsonValue.Create(null));
                            }
                            else
                            {
                                document.Add("FaceBoundingBoxColor", docFaceColor);
                            }

                            if (docFaceGray == null)
                            {
                                document.Add("FaceBoundingBoxGray", BsonValue.Create(null));
                            }
                            else
                            {
                                document.Add("FaceBoundingBoxGray", docFaceGray);
                            }

                            document.Add("WidthColor", BsonValue.Create(frameKinect.WidthColor));
                            document.Add("HeightColor", BsonValue.Create(frameKinect.HeightColor));
                            document.Add("WidthGray", BsonValue.Create(frameKinect.WidthGray));
                            document.Add("HeightGray", BsonValue.Create(frameKinect.HeightGray));
                            document.Add("MaxBodies", BsonValue.Create(frameKinect.MaxBodies));
                            document.Add("MaxJoints", BsonValue.Create(frameKinect.MaxJoints));

                            await tableKinect.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.Zenith)
                        {
                            var frameZenith = (ZenithFrame)data;

                            document.Add("ColorImage", BsonValue.Create(frameZenith.ColorData));
                            document.Add("Width", BsonValue.Create(frameZenith.Width));
                            document.Add("Height", BsonValue.Create(frameZenith.Height));

                            await tableZenith.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.Binary)
                        {
                            var frameBinary = (BinaryFrame)data;

                            document.Add("Value", BsonValue.Create(frameBinary.BinarySensorOutput));

                            await tableBinary.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.WSN)
                        {
                            var frameWSN = (WSNFrame)data;

                            document.Add("Value", BsonValue.Create(frameWSN.WSNOutput));

                            await tableWSN.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.HexiwearBand)
                        {
                            var frameHexiwearBand = (HexiwearBandFrame)data;

                            document.Add("Value", BsonValue.Create(frameHexiwearBand.HexiwearBandOutput));

                            await tableHexiwearBand.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.UPMBand)
                        {
                            var frameUPMBand = (UPMBandFrame)data;

                            document.Add("Value", BsonValue.Create(frameUPMBand.UPMBandOutput));

                            await tableUPMBand.InsertOneAsync(document);
                        }
                        else if (sensorType == SensorType.MSBand)
                        {
                            var frameMSBand = (MSBandFrame)data;

                            document.Add("SensorID", BsonValue.Create(frameMSBand.SensorID));

                            var docAcceleration = new BsonDocument()
                        {
                            { "X", BsonValue.Create(frameMSBand.GyroscopeData.AccelerationX) },
                            { "Y", BsonValue.Create(frameMSBand.GyroscopeData.AccelerationY) },
                            { "Z", BsonValue.Create(frameMSBand.GyroscopeData.AccelerationZ) },
                            { "Time", BsonValue.Create(frameMSBand.GyroscopeData.DataTimeStamp) }
                        };
                            var docCalories = new BsonDocument()
                        {
                            { "Today", BsonValue.Create(frameMSBand.CaloriesData.DataToday) },
                            { "Total", BsonValue.Create(frameMSBand.CaloriesData.DataTotal) },
                            { "Time", BsonValue.Create(frameMSBand.CaloriesData.DataTimeStamp) }
                        };
                            var docDistance = new BsonDocument()
                        {
                            { "Pace", BsonValue.Create(frameMSBand.DistanceData.Pace) },
                            { "Speed", BsonValue.Create(frameMSBand.DistanceData.Speed) },
                            { "Today", BsonValue.Create(frameMSBand.DistanceData.DistanteToday) },
                            { "Time", BsonValue.Create(frameMSBand.DistanceData.DataTimeStamp) }
                        };
                            var docGyroscope = new BsonDocument()
                        {
                            { "VelX", BsonValue.Create(frameMSBand.GyroscopeData.AngularVelocityX) },
                            { "VelY", BsonValue.Create(frameMSBand.GyroscopeData.AngularVelocityY) },
                            { "VelZ", BsonValue.Create(frameMSBand.GyroscopeData.AngularVelocityZ) },
                            { "Time", BsonValue.Create(frameMSBand.GyroscopeData.DataTimeStamp) }
                        };
                            var docHeartRate = new BsonDocument()
                        {
                            { "Value", BsonValue.Create(frameMSBand.HeartRateData.Data) },
                            { "Time", BsonValue.Create(frameMSBand.HeartRateData.DataTimeStamp) }
                        };
                            var docPedometer = new BsonDocument()
                        {
                            { "Today", BsonValue.Create(frameMSBand.PedometerData.StepsToday) },
                            { "Total", BsonValue.Create(frameMSBand.PedometerData.StepsTotal) },
                            { "Time", BsonValue.Create(frameMSBand.PedometerData.DataTimeStamp) }
                        };
                            var docSkinTemp = new BsonDocument()
                        {
                            { "Value", BsonValue.Create(frameMSBand.SkinTemperatureData.Data) },
                            { "Time", BsonValue.Create(frameMSBand.SkinTemperatureData.DataTimeStamp) }
                        };
                            var docGSR = new BsonDocument()
                        {
                            { "Value", BsonValue.Create(frameMSBand.GalvanicSkinResponseData.Gsr) },
                            { "Time", BsonValue.Create(frameMSBand.GalvanicSkinResponseData.DataTimeStamp) }
                        };

                            document.Add("Acceleration", docAcceleration);
                            document.Add("Calories", docCalories);
                            document.Add("Distance", docDistance);
                            document.Add("Gyroscope", docGyroscope);
                            document.Add("HeartRate", docHeartRate);
                            document.Add("Pedometer", docPedometer);
                            document.Add("SkinTemp", docSkinTemp);
                            document.Add("GSR", docGSR);

                            await tableMSBand.InsertOneAsync(document);
                        }
                        else
                        {
                            throw new Exception("This sensor is not supported yet.");
                        }

                        return true;
                    }
                    catch (Exception)
                    {
                        lock (myLock)
                        {
                            ErrorCounter += 1;
                        }

                        return false;
                    }
                }

                /// <summary>Writes the band ids and person ids to the database.</summary>
                public async Task<bool> WriteBandPersonIDs(DateTime start, DateTime end, string session, SensorType sensorType, string sensorID, string personID)
                {
                    if (sensorType != SensorType.MSBand && sensorType != SensorType.HexiwearBand && sensorType != SensorType.UPMBand)
                    {
                        throw new Exception("Only band sensors are valid options (MSBand, HexiwearBand, UPMBand).");
                    }

                    try
                    {
                        var document = new BsonDocument();

                        document.Add("_id", BsonValue.Create(start));
                        document.Add("End", BsonValue.Create(end));
                        document.Add("Session", BsonValue.Create(session));
                        document.Add("SensorType", BsonValue.Create(sensorType.ToString()));
                        document.Add("SensorID", BsonValue.Create(sensorID));
                        document.Add("PersonID", BsonValue.Create(personID));

                        // update existing document or create one
                        var filter = Builders<BsonDocument>.Filter.Eq("_id", BsonValue.Create(start));

                        if (await tableBandPersonIDs.CountAsync(filter) > 0)
                        {
                            await tableBandPersonIDs.FindOneAndReplaceAsync(filter, document);
                        }
                        else
                        {
                            await tableBandPersonIDs.InsertOneAsync(document);
                        }

                        return true;
                    }
                    catch (Exception)
                    {
                        lock (myLock)
                        {
                            ErrorCounter += 1;
                        }

                        return false;
                    }
                }

                /// <summary>Resets the error counter.</summary>
                public void ResetErrors()
                {
                    this.ErrorCounter = 0;
                }

            } // end class

            /// <summary>Provides methods for retrieving data from a MongoDB database.</summary>
            public class DBReader
            {
                protected static IMongoClient client;
                protected static IMongoDatabase database;

                private List<IMongoCollection<BsonDocument>> tables;
                private IMongoCollection<BsonDocument> tableKinect;
                private IMongoCollection<BsonDocument> tableZenith;
                private IMongoCollection<BsonDocument> tableMSBand;
                private IMongoCollection<BsonDocument> tableHexiwearBand;
                private IMongoCollection<BsonDocument> tableUPMBand;
                private IMongoCollection<BsonDocument> tableBinary;
                private IMongoCollection<BsonDocument> tableWSN;
                private IMongoCollection<BsonDocument> tableBandPersonIDs;

                private FilterDefinition<BsonDocument> filter;
                private SortDefinition<BsonDocument> sorting;

                private IAsyncCursor<BsonDocument> docIterator;
                private Queue<BsonDocument> batchResults;

                private BsonDocument eof;

                /// <summary>The database name.</summary>
                public string DatabaseName
                {
                    get; private set;
                }

                /// <summary>Denotes the 'end of file' document (i.e., the database has no more documents for reading).</summary>
                public BsonDocument EoF
                {
                    get { return eof; }
                }


                /// <summary>Connects to the given MongoDB database.</summary>
                public DBReader(string db)
                {
                    DatabaseName = db;

                    client = new MongoClient();
                    database = client.GetDatabase(db);

                    tables = new List<IMongoCollection<BsonDocument>>(SensorUtils.EnumSensors());

                    tableKinect = database.GetCollection<BsonDocument>("Kinect");
                    tableZenith = database.GetCollection<BsonDocument>("Zenith");
                    tableMSBand = database.GetCollection<BsonDocument>("MSBand");
                    tableHexiwearBand = database.GetCollection<BsonDocument>("HexiwearBand");
                    tableUPMBand = database.GetCollection<BsonDocument>("UPMBand");
                    tableBinary = database.GetCollection<BsonDocument>("Binary");
                    tableWSN = database.GetCollection<BsonDocument>("WSN");
                    tableBandPersonIDs = database.GetCollection<BsonDocument>("BandPersonIDs");

                    tables.Add(tableKinect);
                    tables.Add(tableZenith);
                    tables.Add(tableMSBand);
                    tables.Add(tableHexiwearBand);
                    tables.Add(tableUPMBand);
                    tables.Add(tableBinary);
                    tables.Add(tableWSN);

                    sorting = Builders<BsonDocument>.Sort.Ascending("_id");
                    batchResults = new Queue<BsonDocument>();

                    eof = new BsonDocument("EoF", BsonValue.Create(true));
                }

                /// <summary>Checks the connection status.</summary>
                public async Task<bool> IsConnected(int msTimeout = 5000)
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    while (client.Cluster.Description.State != MongoDB.Driver.Core.Clusters.ClusterState.Connected)
                    {
                        await Task.Delay(msTimeout / 4);

                        if (sw.ElapsedMilliseconds > msTimeout)
                        {
                            sw.Stop();

                            return false;
                        }
                    }

                    sw.Stop();

                    return true;
                }

                /// <summary>Returns the available database collections.</summary>
                public async Task<List<string>> GetCollections()
                {
                    List<string> result = new List<string>(10);
                    var collections = await database.ListCollectionsAsync().Result.ToListAsync();

                    foreach (var c in collections)
                    {
                        result.Add(c.GetElement("name").Value.ToString());
                    }

                    return result;
                }

                /// <summary>Clears any filtering criteria.</summary>
                public void ClearFilter()
                {
                    filter = new BsonDocument();
                }

                /// <summary>Enumerates the available documents.</summary>
                public async Task<long> EnumDocuments(SensorType sensorType)
                {
                    filter = new BsonDocument();

                    docIterator = await tables[(int)sensorType].FindAsync(filter, new FindOptions<BsonDocument, BsonDocument> { Sort = sorting });

                    return await tables[(int)sensorType].CountAsync(filter);
                }

                /// <summary>Queries the database for documents that match the provided timespan.</summary>
                public async Task<long> QueryDB(SensorType sensorType, DateTime start, DateTime end)
                {
                    start = new DateTime(start.Ticks, DateTimeKind.Local);
                    end = new DateTime(end.Ticks, DateTimeKind.Local);

                    filter = Builders<BsonDocument>.Filter.Gte("_id", BsonValue.Create(start)) &
                             Builders<BsonDocument>.Filter.Lte("_id", BsonValue.Create(end));

                    docIterator = await tables[(int)sensorType].FindAsync(filter, new FindOptions<BsonDocument, BsonDocument> { Sort = sorting });

                    return await tables[(int)sensorType].CountAsync(filter);
                }

                /// <summary>Queries the database for documents that match the provided session.</summary>
                public async Task<long> QueryDB(SensorType sensorType, string session)
                {
                    filter = Builders<BsonDocument>.Filter.Eq("Session", session);

                    docIterator = await tables[(int)sensorType].FindAsync(filter, new FindOptions<BsonDocument, BsonDocument> { Sort = sorting });

                    return await tables[(int)sensorType].CountAsync(filter);
                }

                /// <summary>Returns the distinct values for the given field.</summary>
                public async Task<List<string>> Distinct(SensorType sensorType, string field)
                {
                    if (filter == null || field == "Session")
                    {
                        filter = new BsonDocument();
                    }

                    var result = await tables[(int)sensorType].DistinctAsync<string>(field, filter);

                    return await result.ToListAsync();
                }

                /// <summary>Enumerates the documents of the sensor table where the field has the given value.</summary>
                public async Task<long> EnumDocumentsWithValue(SensorType sensorType, string field, string value)
                {
                    if (filter == null)
                    {
                        filter = new BsonDocument();
                    }

                    var localFilter = filter & Builders<BsonDocument>.Filter.Eq(field, value);

                    return await tables[(int)sensorType].CountAsync(localFilter);
                }

                /// <summary>Checks if any documents exist where the given field is not null.</summary>
                public async Task<bool> Exists(SensorType sensorType, string field)
                {
                    if (filter == null)
                    {
                        filter = new BsonDocument();
                    }

                    var result = await tables[(int)sensorType].DistinctAsync<dynamic>(field, filter);
                    var list = await result.ToListAsync();

                    return list.Any(p => p != null);
                }

                /// <summary>Retrieves the next document.</summary>
                public async Task<BsonDocument> ReadDocument()
                {
                    if (docIterator == null)
                    {
                        return null;
                    }

                    if (batchResults.Count > 0)
                    {
                        return batchResults.Dequeue();
                    }
                    else
                    {
                        if (await docIterator.MoveNextAsync())
                        {
                            foreach (var doc in docIterator.Current)
                            {
                                batchResults.Enqueue(doc);
                            }

                            if (batchResults.Count == 0)
                            {
                                return EoF;
                            }

                            return batchResults.Dequeue();
                        }
                        else
                        {
                            return EoF;
                        }
                    }
                }

                /// <summary>Retrieves the next document from the specified sensor.</summary>
                public async Task<BsonDocument> ReadDocument(string sensorID)
                {
                    BsonDocument doc = await ReadDocument();

                    while (doc != EoF && doc["SensorID"].AsString != sensorID)
                    {
                        doc = await ReadDocument();
                    }

                    return doc;
                }

                /// <summary>Updates the database labels.</summary>
                public async Task<Dictionary<SensorType, int>> UpdateDocument(DateTime start, DateTime end, string label, string PersonID, string KinectID, string KinectBodyID)
                {
                    var results = SensorUtils.SensorDictionary(0);

                    start = new DateTime(start.Ticks, DateTimeKind.Local);
                    end = new DateTime(end.Ticks, DateTimeKind.Local);

                    var diff = start.Subtract(start.ToUniversalTime());
                    var filter = Builders<BsonDocument>.Filter.Gte("_id", BsonValue.Create(start + diff)) &
                                 Builders<BsonDocument>.Filter.Lte("_id", BsonValue.Create(end + diff));

                    var docLabelOther = Builders<BsonDocument>.Update.AddToSet("Label", label);
                    var docPersonID = Builders<BsonDocument>.Update.Set("PersonID", PersonID);

                    UpdateResult r;

                    if (PersonID != "" && KinectID != "" && KinectBodyID != "-1")
                    {
                        var filterKinect = filter & Builders<BsonDocument>.Filter.Eq("SensorID", BsonValue.Create(KinectID));

                        var docLabelKinect = Builders<BsonDocument>.Update.AddToSet("Label", new string[] { KinectBodyID, label });
                        var docPersonIDKinect = Builders<BsonDocument>.Update.AddToSet("PersonID", new string[] { KinectBodyID, PersonID });

                        r = await tableKinect.UpdateManyAsync(filterKinect, docLabelKinect);
                        await tableKinect.UpdateManyAsync(filterKinect, docPersonIDKinect);

                        if (r.IsModifiedCountAvailable)
                        {
                            results[SensorType.Kinect] = (int)r.ModifiedCount;
                        }
                    }

                    if (PersonID != "")
                    {
                        var BandPersonFilter = Builders<BsonDocument>.Filter.Not(Builders<BsonDocument>.Filter.Gt("_id", BsonValue.Create(end + diff)) |
                                                                                 Builders<BsonDocument>.Filter.Lt("End", BsonValue.Create(start + diff)));

                        var UserIDFilter = Builders<BsonDocument>.Filter.Eq("PersonID", BsonValue.Create(PersonID));

                        using (var iter = await tableBandPersonIDs.FindAsync(BandPersonFilter & UserIDFilter))
                        {
                            while (await iter.MoveNextAsync())
                            {
                                var batch = iter.Current;
                                foreach (var doc in batch)
                                {
                                    var bpStart = doc["_id"].ToUniversalTime();
                                    var bpEnd = doc["End"].ToUniversalTime();

                                    if (start < bpStart)
                                    {
                                        start = bpStart;
                                    }
                                    if (end > bpEnd)
                                    {
                                        end = bpEnd;
                                    }

                                    start = new DateTime(start.Ticks, DateTimeKind.Local);
                                    end = new DateTime(end.Ticks, DateTimeKind.Local);

                                    filter = Builders<BsonDocument>.Filter.Gte("_id", BsonValue.Create(start + diff)) &
                                             Builders<BsonDocument>.Filter.Lte("_id", BsonValue.Create(end + diff));

                                    if (doc["SensorType"].AsString == SensorType.MSBand.ToString())
                                    {
                                        r = await tableMSBand.UpdateManyAsync(filter, docLabelOther);
                                        await tableMSBand.UpdateManyAsync(filter, docPersonID);

                                        if (r.IsModifiedCountAvailable)
                                        {
                                            results[SensorType.MSBand] = (int)r.ModifiedCount;
                                        }
                                    }
                                    else if (doc["SensorType"].AsString == SensorType.UPMBand.ToString())
                                    {
                                        r = await tableUPMBand.UpdateManyAsync(filter, docLabelOther);
                                        await tableUPMBand.UpdateManyAsync(filter, docPersonID);

                                        if (r.IsModifiedCountAvailable)
                                        {
                                            results[SensorType.UPMBand] = (int)r.ModifiedCount;
                                        }
                                    }
                                    else if (doc["SensorType"].AsString == SensorType.HexiwearBand.ToString())
                                    {
                                        r = await tableHexiwearBand.UpdateManyAsync(filter, docLabelOther);
                                        await tableHexiwearBand.UpdateManyAsync(filter, docPersonID);

                                        if (r.IsModifiedCountAvailable)
                                        {
                                            results[SensorType.HexiwearBand] = (int)r.ModifiedCount;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    return results;
                }

                /// <summary>Retrieves the Kinect body frame.</summary>
                public DataFrames.Kinect.Body[] ExtractBodyFrame(BsonDocument doc)
                {
                    KinectFrame kf = new KinectFrame();

                    for (int i = 0; i < kf.MaxBodies; i++)
                    {
                        if (doc != null && doc["BodyFrame"][i]["skeleton"]["raw"] != BsonNull.Value)
                        {
                            kf.BodyFrame[i].firstTrack = doc["BodyFrame"][i]["firstTrack"].AsBoolean;
                            kf.BodyFrame[i].isTracked = doc["BodyFrame"][i]["isTracked"].AsBoolean;
                            kf.BodyFrame[i].leanLR = (float)doc["BodyFrame"][i]["leanLR"].AsDouble;
                            kf.BodyFrame[i].leanFB = (float)doc["BodyFrame"][i]["leanFB"].AsDouble;
                            kf.BodyFrame[i].leanConfidence = doc["BodyFrame"][i]["leanConfidence"].AsInt32;

                            for (int j = 0; j < kf.MaxJoints; j++)
                            {
                                kf.BodyFrame[i].skeletonData.jointsRaw[j].x = (float)doc["BodyFrame"][i]["skeleton"]["raw"][j]["x"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRaw[j].y = (float)doc["BodyFrame"][i]["skeleton"]["raw"][j]["y"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRaw[j].z = (float)doc["BodyFrame"][i]["skeleton"]["raw"][j]["z"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRaw[j].confidence = doc["BodyFrame"][i]["skeleton"]["raw"][j]["confidence"].AsInt32;

                                kf.BodyFrame[i].skeletonData.jointsRawColor[j].x = (float)doc["BodyFrame"][i]["skeleton"]["rawColor"][j]["x"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRawColor[j].y = (float)doc["BodyFrame"][i]["skeleton"]["rawColor"][j]["y"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRawColor[j].z = (float)doc["BodyFrame"][i]["skeleton"]["rawColor"][j]["z"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRawColor[j].confidence = doc["BodyFrame"][i]["skeleton"]["rawColor"][j]["confidence"].AsInt32;

                                kf.BodyFrame[i].skeletonData.jointsRawGray[j].x = (float)doc["BodyFrame"][i]["skeleton"]["rawGray"][j]["x"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRawGray[j].y = (float)doc["BodyFrame"][i]["skeleton"]["rawGray"][j]["y"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRawGray[j].z = (float)doc["BodyFrame"][i]["skeleton"]["rawGray"][j]["z"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsRawGray[j].confidence = doc["BodyFrame"][i]["skeleton"]["rawGray"][j]["confidence"].AsInt32;

                                kf.BodyFrame[i].skeletonData.jointsFiltered[j].x = (float)doc["BodyFrame"][i]["skeleton"]["filtered"][j]["x"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFiltered[j].y = (float)doc["BodyFrame"][i]["skeleton"]["filtered"][j]["y"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFiltered[j].z = (float)doc["BodyFrame"][i]["skeleton"]["filtered"][j]["z"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFiltered[j].confidence = doc["BodyFrame"][i]["skeleton"]["filtered"][j]["confidence"].AsInt32;

                                kf.BodyFrame[i].skeletonData.jointsFilteredColor[j].x = (float)doc["BodyFrame"][i]["skeleton"]["filteredColor"][j]["x"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFilteredColor[j].y = (float)doc["BodyFrame"][i]["skeleton"]["filteredColor"][j]["y"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFilteredColor[j].z = (float)doc["BodyFrame"][i]["skeleton"]["filteredColor"][j]["z"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFilteredColor[j].confidence = doc["BodyFrame"][i]["skeleton"]["filteredColor"][j]["confidence"].AsInt32;

                                kf.BodyFrame[i].skeletonData.jointsFilteredGray[j].x = (float)doc["BodyFrame"][i]["skeleton"]["filteredGray"][j]["x"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFilteredGray[j].y = (float)doc["BodyFrame"][i]["skeleton"]["filteredGray"][j]["y"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFilteredGray[j].z = (float)doc["BodyFrame"][i]["skeleton"]["filteredGray"][j]["z"].AsDouble;
                                kf.BodyFrame[i].skeletonData.jointsFilteredGray[j].confidence = doc["BodyFrame"][i]["skeleton"]["filteredGray"][j]["confidence"].AsInt32;
                            }
                        }
                    }

                    return kf.BodyFrame;
                }

            } // end class

            /// <summary>Provides methods for creating video files.</summary>
            public class VideoWriter
            {
                Emgu.CV.VideoWriter writer;

                FrameOperations.ColorTypes type;
                int width;
                int height;

                /// <summary>The number of errors during writing.</summary>
                public int ErrorCounter
                {
                    get; private set;
                }


                /// <summary>Creates an empty video with the specified parameters.</summary>
                public VideoWriter(string filename, int imageWidth, int imageHeight, FrameOperations.ColorTypes cType, int fps)
                {
                    this.width = imageWidth;
                    this.height = imageHeight;
                    this.type = cType;

                    this.ErrorCounter = 0;

                    // delete old video
                    try
                    {
                        File.Delete(filename);
                    }
                    catch (Exception)
                    {

                    }

                    if (type == FrameOperations.ColorTypes.Bgra || type == FrameOperations.ColorTypes.Bgr)
                    {
                        this.writer = new Emgu.CV.VideoWriter(filename, -1, fps, new Size(width, height), true);
                    }
                    else
                    {
                        this.writer = new Emgu.CV.VideoWriter(filename, Emgu.CV.VideoWriter.Fourcc('X', '2', '6', '4'), fps, new Size(width, height), true);
                    }
                }

                /// <summary>Adds a frame to the video.</summary>
                public bool WriteFrame(byte[] data, string info = null)
                {
                    try
                    {
                        if (type == FrameOperations.ColorTypes.Bgra)
                        {
                            if (data.Length != width * height * 4 || data.Length == 0)
                            {
                                this.ErrorCounter += 1;

                                return false;
                            }

                            var frame = FrameOperations.Convert.BytesToImageBgra(data, width, height);

                            if (info != null)
                            {
                                CvInvoke.PutText(frame, info, new Point(20, height - 20), Emgu.CV.CvEnum.FontFace.HersheySimplex, 0.5, new Bgra(255, 255, 255, 155).MCvScalar);
                            }

                            writer.Write(frame.Mat);
                        }
                        else if (type == FrameOperations.ColorTypes.Bgr)
                        {
                            if (data.Length != width * height * 3 || data.Length == 0)
                            {
                                this.ErrorCounter += 1;

                                return false;
                            }

                            var frame = FrameOperations.Convert.BytesToImageBgr(data, width, height);

                            if (info != null)
                            {
                                CvInvoke.PutText(frame, info, new Point(20, height - 20), Emgu.CV.CvEnum.FontFace.HersheyPlain, 1, new Bgr(255, 255, 255).MCvScalar);
                            }

                            writer.Write(frame.Mat);
                        }
                        else if (type == FrameOperations.ColorTypes.Gray16)
                        {
                            if (data.Length != width * height * 2 || data.Length == 0)
                            {
                                this.ErrorCounter += 1;

                                return false;
                            }

                            var frame = FrameOperations.Convert.BytesToImageGray16(data, width, height);

                            writer.Write(frame.Mat);
                        }
                        else // type == FrameOperations.ColorTypes.Gray8
                        {
                            if (data.Length != width * height || data.Length == 0)
                            {
                                this.ErrorCounter += 1;

                                return false;
                            }

                            var frame = FrameOperations.Convert.BytesToImageGray8(data, width, height);

                            writer.Write(frame.Mat);
                        }

                        return true;
                    }
                    catch (Exception)
                    {
                        this.ErrorCounter += 1;

                        return false;
                    }
                }

                /// <summary>Closes the video file and releases resources.</summary>
                public void Close()
                {
                    try
                    {
                        writer.Dispose();
                    }
                    catch (Exception)
                    {

                    }
                }

            } // end class

            /// <summary>Saves the given data as a JPEG image.</summary>
            public static void SaveJpeg(string fileName, byte[] frame, int width, int height, PixelFormat format, int quality)
            {
                string dir = Path.GetDirectoryName(fileName);
                if (Directory.Exists(dir) == false)
                {
                    Directory.CreateDirectory(dir);
                }

                using (FileStream fs = new FileStream(fileName, FileMode.Create))
                {
                    JpegBitmapEncoder encoder = new JpegBitmapEncoder();
                    encoder.QualityLevel = quality;

                    WriteableBitmap wb = FrameOperations.Convert.BytesToBitmap(frame, width, height, format);

                    encoder.Frames.Add(BitmapFrame.Create(wb));
                    encoder.Save(fs);
                }
            }

            /// <summary>Saves the given data as a JPEG image.</summary>
            public static void SaveJpeg(string fileName, WriteableBitmap frame, int quality)
            {
                string dir = Path.GetDirectoryName(fileName);
                if (Directory.Exists(dir) == false)
                {
                    Directory.CreateDirectory(dir);
                }

                using (FileStream fs = new FileStream(fileName, FileMode.Create))
                {
                    JpegBitmapEncoder encoder = new JpegBitmapEncoder();
                    encoder.QualityLevel = quality;

                    encoder.Frames.Add(BitmapFrame.Create(frame));
                    encoder.Save(fs);
                }
            }

            /// <summary>Saves the given data as a PNG image.</summary>
            public static void SavePng(string fileName, byte[] frame, int width, int height, PixelFormat format)
            {
                string dir = Path.GetDirectoryName(fileName);
                if (Directory.Exists(dir) == false)
                {
                    Directory.CreateDirectory(dir);
                }

                using (FileStream fs = new FileStream(fileName, FileMode.Create))
                {
                    PngBitmapEncoder encoder = new PngBitmapEncoder();
                    encoder.Interlace = PngInterlaceOption.Off;

                    WriteableBitmap wb = FrameOperations.Convert.BytesToBitmap(frame, width, height, format);

                    encoder.Frames.Add(BitmapFrame.Create(wb));
                    encoder.Save(fs);
                }
            }

            /// <summary>Saves the given data as a PNG image.</summary>
            public static void SavePng(string fileName, WriteableBitmap frame)
            {
                string dir = Path.GetDirectoryName(fileName);
                if (Directory.Exists(dir) == false)
                {
                    Directory.CreateDirectory(dir);
                }

                using (FileStream fs = new FileStream(fileName, FileMode.Create))
                {
                    PngBitmapEncoder encoder = new PngBitmapEncoder();
                    encoder.Interlace = PngInterlaceOption.Off;

                    encoder.Frames.Add(BitmapFrame.Create(frame));
                    encoder.Save(fs);
                }
            }

            /// <summary>Saves the given data as a binary file.</summary>
            public static void Save(string fileName, byte[] data)
            {
                string dir = Path.GetDirectoryName(fileName);
                if (Directory.Exists(dir) == false)
                {
                    Directory.CreateDirectory(dir);
                }

                using (FileStream fs = new FileStream(fileName, FileMode.Create))
                {
                    fs.Write(data, 0, data.Length);
                }
            }

            /// <summary>Loads an image from disk.</summary>
            public static byte[] LoadImage(string filename, FrameOperations.ColorTypes cType, out int width, out int height)
            {
                byte[] result;

                if (cType == FrameOperations.ColorTypes.Bgra)
                {
                    Image<Bgra, byte> img = new Image<Bgra, byte>(filename);
                    result = img.Bytes;

                    width = img.Width;
                    height = img.Height;
                }
                else if (cType == FrameOperations.ColorTypes.Bgr)
                {
                    Image<Bgr, byte> img = new Image<Bgr, byte>(filename);
                    result = img.Bytes;

                    width = img.Width;
                    height = img.Height;
                }
                else if (cType == FrameOperations.ColorTypes.Gray16)
                {
                    Image<Gray, Int16> img = new Image<Gray, Int16>(filename);
                    result = img.Bytes;

                    width = img.Width;
                    height = img.Height;
                }
                else if (cType == FrameOperations.ColorTypes.Gray8)
                {
                    Image<Gray, byte> img = new Image<Gray, byte>(filename);
                    result = img.Bytes;

                    width = img.Width;
                    height = img.Height;
                }
                else
                {
                    throw new Exception("This color type is not supported.");
                }

                return result;
            }

        } // end class

        /// <summary>Provides methods for manipulating images and coordinates.</summary>
        public static class FrameOperations
        {

            /// <summary>The supported color formats.</summary>
            public enum ColorTypes
            {
                /// <summary>Denotes the Blue-Green-Red-Alpha color format.</summary>
                Bgra = 0,
                /// <summary>Denotes the Blue-Green-Red color format.</summary>
                Bgr = 1,
                /// <summary>Denotes the 2-channel (int16) gray-scale color format.</summary>
                Gray16 = 2,
                /// <summary>Denotes the single channel gray-scale color format.</summary>
                Gray8 = 3
            }

            /// <summary>Provides methods for transforming 3D coordinates into different coordinate systems.</summary>
            public class TransformCoordinates
            {
                private Matrix<double> _rotation;
                private Matrix<double> _translation;


                /// <summary>The 3x3 rotation matrix.</summary>
                public Matrix<double> Rotation
                {
                    get { return _rotation; }
                    private set { _rotation = value; }
                }

                /// <summary>The 3x1 translation matrix.</summary>
                public Matrix<double> Translation
                {
                    get { return _translation; }
                    private set { _translation = value; }
                }


                /// <summary>Loads the Rotation and Translation matrices from the given file.</summary>
                public TransformCoordinates(string filename)
                {
                    Rotation = new Matrix<double>(3, 3);
                    Translation = new Matrix<double>(3, 1);

                    string[] data = File.ReadAllLines(filename);
                    string[] values;

                    for (int i = 0; i < 3; i++)
                    {
                        values = data[i].Split(',');
                        for (int j = 0; j < 3; j++)
                        {
                            Rotation[j, i] = double.Parse(values[j]);
                        }
                    }

                    values = data[3].Split(',');
                    for (int j = 0; j < 3; j++)
                    {
                        Translation[j, 0] = double.Parse(values[j]);
                    }
                }

                /// <summary>Transforms the given 3D point (x, y, z) into the new coordinate system.</summary>
                public double[,] Transform(double x, double y, double z)
                {
                    var newPoint = new Matrix<double>(3, 1);
                    newPoint[0, 0] = x;
                    newPoint[1, 0] = y;
                    newPoint[2, 0] = z;

                    newPoint = Rotation * newPoint;
                    newPoint += Translation;

                    return newPoint.Data;
                }

                /// <summary>Transforms the given skeleton joint into the new coordinate system.</summary>
                public SkeletonJoint Transform(SkeletonJoint joint)
                {
                    var newPoint = new Matrix<double>(3, 1);
                    newPoint[0, 0] = joint.x;
                    newPoint[1, 0] = joint.y;
                    newPoint[2, 0] = joint.z;

                    newPoint = Rotation * newPoint;
                    newPoint += Translation;

                    var newJoint = new SkeletonJoint();
                    newJoint.x = (float)newPoint[0, 0];
                    newJoint.y = (float)newPoint[1, 0];
                    newJoint.z = (float)newPoint[2, 0];
                    newJoint.confidence = joint.confidence;

                    return newJoint;
                }

                /// <summary>Transforms the given skeleton joints into the new coordinate system.</summary>
                public SkeletonJoint[] Transform(SkeletonJoint[] joints)
                {
                    var newJoints = new List<SkeletonJoint>(joints.Length);

                    var newPoint = new Matrix<double>(3, 1);
                    var newJoint = new SkeletonJoint();

                    foreach (var joint in joints)
                    {
                        newPoint[0, 0] = joint.x;
                        newPoint[1, 0] = joint.y;
                        newPoint[2, 0] = joint.z;

                        newPoint = Rotation * newPoint;
                        newPoint += Translation;

                        newJoint.x = (float)newPoint[0, 0];
                        newJoint.y = (float)newPoint[1, 0];
                        newJoint.z = (float)newPoint[2, 0];
                        newJoint.confidence = joint.confidence;

                        newJoints.Add(newJoint);
                    }

                    return newJoints.ToArray();
                }

            } // end class

            /// <summary>Provides image visualization functionality.</summary>
            public static class Display
            {

                /// <summary>Creates a window and displays the given image.</summary>
                public static void Show(byte[] data, int width, int height, ColorTypes cType, int wait = 25, string windowName = "Display")
                {
                    if (cType == ColorTypes.Bgra)
                    {
                        var image = Convert.BytesToImageBgra(data, width, height);

                        CvInvoke.Imshow(windowName, image);
                        CvInvoke.WaitKey(wait);
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        var image = Convert.BytesToImageBgr(data, width, height);

                        CvInvoke.Imshow(windowName, image);
                        CvInvoke.WaitKey(wait);
                    }
                    else if (cType == ColorTypes.Gray16)
                    {
                        var image = Convert.BytesToImageGray16(data, width, height);

                        CvInvoke.Imshow(windowName, image);
                        CvInvoke.WaitKey(wait);
                    }
                    else if (cType == ColorTypes.Gray8)
                    {
                        var image = Convert.BytesToImageGray8(data, width, height);

                        CvInvoke.Imshow(windowName, image);
                        CvInvoke.WaitKey(wait);
                    }
                    else
                    {
                        throw new Exception("ColorType must be one of Bgra, Bgr, Gray16 and Gray8.");
                    }
                }

                /// <summary>Closes a created window by name.</summary>
                public static void CloseWindow(string name)
                {
                    CvInvoke.DestroyWindow(name);
                }

            }

            /// <summary>Provides methods for converting raw bytes into different image types.</summary>
            public static class Convert
            {

                /// <summary>Converts bytes into a BGRA image.</summary>
                public static Image<Bgra, byte> BytesToImageBgra(byte[] data, int width, int height)
                {
                    Image<Bgra, byte> result = new Image<Bgra, byte>(width, height);
                    result.Bytes = data;

                    return result;
                }

                /// <summary>Converts bytes into a BGR image.</summary>
                public static Image<Bgr, byte> BytesToImageBgr(byte[] data, int width, int height)
                {
                    Image<Bgr, byte> result = new Image<Bgr, byte>(width, height);
                    result.Bytes = data;

                    return result;
                }

                /// <summary>Converts bytes into a Gray16 image.</summary>
                public static Image<Gray, Int16> BytesToImageGray16(byte[] data, int width, int height)
                {
                    Image<Gray, Int16> result = new Image<Gray, Int16>(width, height);
                    result.Bytes = data;

                    return result;
                }

                /// <summary>Converts bytes into a Gray image.</summary>
                public static Image<Gray, byte> BytesToImageGray8(byte[] data, int width, int height)
                {
                    Image<Gray, byte> result = new Image<Gray, byte>(width, height);
                    result.Bytes = data;

                    return result;
                }

                /// <summary>Converts bytes into a WriteableBitmap.</summary>
                public static WriteableBitmap BytesToBitmap(byte[] data, int width, int height, PixelFormat format)
                {
                    int stride = width * ((format.BitsPerPixel + 7) / 8);
                    WriteableBitmap wb = new WriteableBitmap(width, height, 96.0, 96.0, format, null);
                    wb.WritePixels(new System.Windows.Int32Rect(0, 0, width, height), data, stride, 0);

                    return wb;
                }

                /// <summary>Converts bytes that represent a BGRA or BGR image into bytes that represent a Gray image.</summary>
                public static byte[] ColorToGray(byte[] data, int width, int height, ColorTypes cType)
                {
                    if (cType == ColorTypes.Bgra)
                    {
                        var before = BytesToImageBgra(data, width, height);
                        Image<Gray, byte> result = new Image<Gray, byte>(width, height);

                        CvInvoke.CvtColor(before, result, Emgu.CV.CvEnum.ColorConversion.Bgra2Gray);

                        return result.Bytes;
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        var before = BytesToImageBgr(data, width, height);
                        Image<Gray, byte> result = new Image<Gray, byte>(width, height);

                        CvInvoke.CvtColor(before, result, Emgu.CV.CvEnum.ColorConversion.Bgr2Gray);

                        return result.Bytes;
                    }
                    else
                    {
                        throw new Exception("ColorType must be Bgra or Bgr.");
                    }
                }

            }

            /// <summary>Provides methods for data compression.</summary>
            public static class Compress
            {

                /// <summary>Compresses the given bytes with the JPEG algorithm (Lossy).</summary>
                public static byte[] CompressJPEG(byte[] data, int width, int height, PixelFormat format, int quality)
                {
                    using (MemoryStream memoryStream = new MemoryStream())
                    {
                        JpegBitmapEncoder encoder = new JpegBitmapEncoder();
                        encoder.QualityLevel = quality;

                        int stride = width * ((format.BitsPerPixel + 7) / 8);
                        encoder.Frames.Add(BitmapFrame.Create(BitmapSource.Create(width, height, 96, 96, format, null, data, stride)));
                        encoder.Save(memoryStream);

                        return memoryStream.ToArray();
                    }
                }

                /// <summary>Compresses the given bytes with the PNG algorithm (Lossless).</summary>
                public static byte[] CompressPNG(byte[] data, int width, int height, PixelFormat format)
                {
                    using (MemoryStream memoryStream = new MemoryStream())
                    {
                        PngBitmapEncoder encoder = new PngBitmapEncoder();
                        encoder.Interlace = PngInterlaceOption.Off;

                        int stride = width * ((format.BitsPerPixel + 7) / 8);
                        encoder.Frames.Add(BitmapFrame.Create(BitmapSource.Create(width, height, 96, 96, format, null, data, stride)));
                        encoder.Save(memoryStream);

                        return memoryStream.ToArray();
                    }
                }

                /// <summary>Compresses the given bytes with Facebook's ZSTD algorithm (Lossless).</summary>
                public static byte[] CompressZSTD(byte[] data, int quality, byte[] dictionary = null)
                {
                    CompressionOptions opt;
                    if (quality >= 1 && quality <= 22)
                    {
                        opt = new CompressionOptions(dictionary, quality);
                    }
                    else
                    {
                        opt = new CompressionOptions(3);
                        Console.WriteLine("CompressZSTD: quality is out of range");
                    }

                    using (Compressor comp = new Compressor(opt))
                    {
                        return comp.Wrap(data);
                    }
                }

                /// <summary>Builds a dictionary for the ZSTD algorithm.</summary>
                public static byte[] BuildDictionaryZSTD(List<byte[]> data, int size = -1)
                {
                    if (size == -1)
                    {
                        size = 0;

                        foreach (var item in data)
                        {
                            size += item.Length;
                        }
                    }

                    return DictBuilder.TrainFromBuffer(data, size);
                }

                /// <summary>Reads a dictionary for the ZSTD algorithm.</summary>
                public static byte[] ReadDictionaryZSTD(string filename)
                {
                    return File.ReadAllBytes(filename);
                }

                /// <summary>Saves a dictionary for the ZSTD algorithm.</summary>
                public static void SaveDictionaryZSTD(string filename, byte[] dictionary)
                {
                    File.WriteAllBytes(filename, dictionary);
                }

                /// <summary>Compresses color images using JPEG and gray images using ZSTD.</summary>
                public static byte[] CompressImage(byte[] data, int width, int height, ColorTypes colorType, int quality)
                {
                    if (colorType == ColorTypes.Bgra)
                    {
                        if (quality <= 100)
                        {
                            return CompressJPEG(data, width, height, PixelFormats.Bgra32, quality);
                        }
                        else
                        {
                            return CompressZSTD(data, 1);
                        }
                    }
                    else if (colorType == ColorTypes.Bgr)
                    {
                        if (quality <= 100)
                        {
                            return CompressJPEG(data, width, height, PixelFormats.Bgr24, quality);
                        }
                        else
                        {
                            return CompressZSTD(data, 1);
                        }
                    }
                    else if (colorType == ColorTypes.Gray16)
                    {
                        if (quality > 0)
                        {
                            data = Transform.ReduceFidelity(data, width, height, quality);
                        }

                        return CompressZSTD(data, 2);
                    }
                    else // (colorType == ColorTypes.Gray8)
                    {
                        return CompressZSTD(data, 2);
                    }
                }

            }

            /// <summary>Provides methods for data decompression.</summary>
            public static class Decompress
            {

                /// <summary>Decompresses the given bytes with the JPEG algorithm.</summary>
                public static byte[] DecompressJPEG(byte[] data, ref int width, ref int height, PixelFormat format)
                {
                    using (MemoryStream memoryStream = new MemoryStream(data))
                    {
                        JpegBitmapDecoder decoder;
                        if (format == PixelFormats.Bgra32)
                        {
                            decoder = new JpegBitmapDecoder(memoryStream, BitmapCreateOptions.None, BitmapCacheOption.Default);
                        }
                        else // Bgr24
                        {
                            decoder = new JpegBitmapDecoder(memoryStream, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
                        }

                        BitmapFrame frame = decoder.Frames[0];

                        if (width != frame.Width || height != frame.Height)
                        {
                            width = (int)frame.Width;
                            height = (int)frame.Height;
                        }

                        int stride = width * ((format.BitsPerPixel + 7) / 8);
                        byte[] result = new byte[height * stride];

                        frame.CopyPixels(result, stride, 0);

                        return result;
                    }
                }

                /// <summary>Decompresses the given bytes with the PNG algorithm.</summary>
                public static byte[] DecompressPNG(byte[] data, ref int width, ref int height, PixelFormat format)
                {
                    using (MemoryStream memoryStream = new MemoryStream(data))
                    {
                        PngBitmapDecoder decoder = new PngBitmapDecoder(memoryStream, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);

                        BitmapFrame frame = decoder.Frames[0];

                        if (width != frame.Width || height != frame.Height)
                        {
                            width = (int)frame.Width;
                            height = (int)frame.Height;
                        }

                        int stride = width * ((format.BitsPerPixel + 7) / 8);
                        byte[] result = new byte[height * stride];

                        frame.CopyPixels(result, stride, 0);

                        return result;
                    }
                }

                /// <summary>Decompresses the given bytes with Facebook's ZSTD algorithm.</summary>
                public static byte[] DecompressZSTD(byte[] data, byte[] dictionary = null)
                {
                    DecompressionOptions opt = new DecompressionOptions(dictionary);
                    using (Decompressor decomp = new Decompressor(opt))
                    {
                        return decomp.Unwrap(data);
                    }
                }

                /// <summary>Decompresses image data using the appropriate algorithm inferred from the metadata.</summary>
                public static byte[] DecompressImage(byte[] data, ref int width, ref int height, ColorTypes colorType)
                {
                    if (colorType == ColorTypes.Bgra)
                    {
                        if (IsJPEG(data))
                        {
                            return DecompressJPEG(data, ref width, ref height, PixelFormats.Bgra32);
                        }
                        else
                        {
                            return DecompressZSTD(data);
                        }
                    }
                    else if (colorType == ColorTypes.Bgr)
                    {
                        if (IsJPEG(data))
                        {
                            return DecompressJPEG(data, ref width, ref height, PixelFormats.Bgr24);
                        }
                        else
                        {
                            return DecompressZSTD(data);
                        }
                    }
                    else if (colorType == ColorTypes.Gray16)
                    {
                        return DecompressZSTD(data);
                    }
                    else // (colorType == ColorTypes.Gray8)
                    {
                        return DecompressZSTD(data);
                    }
                }

                /// <summary>Determines if the given bytes correspond to a JPEG compressed image.</summary>
                public static bool IsJPEG(byte[] data)
                {
                    return data[0] == 255 && data[1] == 216 && data[2] == 255 && data[3] == 224 && data[4] == 0; // JFIF header
                }

            }

            /// <summary>Provides image transformation methods.</summary>
            public static class Transform
            {

                /// <summary>Computes the mean absolute error (MAE) between two images.</summary>
                public static double MeanAbsoluteError(byte[] data1, byte[] data2, int width, int height, ColorTypes cType)
                {
                    double result = -1;

                    if (cType == ColorTypes.Bgra)
                    {
                        var im1 = Convert.BytesToImageBgra(data1, width, height);
                        var im2 = Convert.BytesToImageBgra(data2, width, height);
                        var sum = (im1.AbsDiff(im2)).GetSum();

                        result = (sum.Blue + sum.Green + sum.Red + sum.Alpha) / (width * height);
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        var im1 = Convert.BytesToImageBgr(data1, width, height);
                        var im2 = Convert.BytesToImageBgr(data2, width, height);
                        var sum = (im1.AbsDiff(im2)).GetSum();

                        result = (sum.Blue + sum.Green + sum.Red) / (width * height);
                    }
                    else if (cType == ColorTypes.Gray16)
                    {
                        var im1 = Convert.BytesToImageGray16(data1, width, height);
                        var im2 = Convert.BytesToImageGray16(data2, width, height);
                        var sum = (im1.AbsDiff(im2)).GetSum();

                        result = sum.Intensity / (width * height);
                    }
                    else // (cType == ColorTypes.Gray8)
                    {
                        var im1 = Convert.BytesToImageGray8(data1, width, height);
                        var im2 = Convert.BytesToImageGray8(data2, width, height);
                        var sum = (im1.AbsDiff(im2)).GetSum();

                        result = sum.Intensity / (width * height);
                    }

                    return result;
                }

                /// <summary>Deletes the alpha channel from a BGRA image.</summary>
                public static byte[] DeleteAlphaChannel(byte[] data, int width, int height)
                {
                    var before = Convert.BytesToImageBgra(data, width, height);
                    Image<Bgr, byte> result = new Image<Bgr, byte>(width, height);

                    CvInvoke.CvtColor(before, result, Emgu.CV.CvEnum.ColorConversion.Bgra2Bgr, 3);

                    return result.Bytes;
                }

                /// <summary>Reduces the fidelity of a gray16 image in such a way that each pixel in the resulting image is changed by at most maxError.</summary>
                public static byte[] ReduceFidelity(byte[] data, int width, int height, int maxError)
                {
                    var result = Convert.BytesToImageGray16(data, width, height);
                    int value = 2 * maxError;

                    if (Validation.ValidateNumber(value, 1, int.MaxValue - 1) == false)
                    {
                        return data;
                    }

                    result /= value;
                    result *= value;

                    return result.Bytes;
                }

                /// <summary>Performs gamma correction on the given gray16 image.</summary>
                public static byte[] GammaCorrection(byte[] data, int width, int height, double gamma)
                {
                    var result = Convert.BytesToImageGray16(data, width, height);
                    var resultByte = result.ConvertScale<byte>(1.0 / 255, 0);

                    resultByte._GammaCorrect(gamma);
                    result = resultByte.ConvertScale<Int16>(255, 0);

                    return result.Bytes;
                }

                /// <summary>Draws the Kinect skeleton onto a color (BGRA or BGR) image.</summary>
                public static byte[] SkeletonToColor(KinectFrame frame, ColorTypes cType, bool filtered, int jointSize)
                {
                    Bgra[] colorsBgra = { new Bgra(0, 0, 255, 255),
                                      new Bgra(0, 255, 0, 255),
                                      new Bgra(255, 0, 0, 255),
                                      new Bgra(255, 0, 255, 255),
                                      new Bgra(0, 255, 255, 255),
                                      new Bgra(255, 255, 0, 255) };

                    Bgr[] colorsBgr = { new Bgr(0, 0, 255),
                                    new Bgr(0, 255, 0),
                                    new Bgr(255, 0, 0),
                                    new Bgr(255, 0, 255),
                                    new Bgr(0, 255, 255),
                                    new Bgr(255, 255, 0) };

                    int[] joints = { 3, 20, 8, 9, 10, 4, 5, 6, 1, 0, 16, 17, 18, 19, 12, 13, 14, 15 };

                    Rectangle boundsColor = new Rectangle(0, 0, frame.WidthColor - 1, frame.HeightColor - 1);

                    object image;
                    if (cType == ColorTypes.Bgra)
                    {
                        image = Convert.BytesToImageBgra(frame.ColorImage, frame.WidthColor, frame.HeightColor);
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        image = Convert.BytesToImageBgr(frame.ColorImage, frame.WidthColor, frame.HeightColor);
                    }
                    else
                    {
                        throw new Exception("cType must be Bgra or Bgr.");
                    }

                    for (int i = 0; i < frame.MaxBodies; i++)
                    {
                        if (frame.BodyFrame[i].isTracked)
                        {
                            SkeletonJoint[] sj;
                            if (filtered)
                            {
                                sj = frame.BodyFrame[i].skeletonData.jointsFilteredColor;
                            }
                            else
                            {
                                sj = frame.BodyFrame[i].skeletonData.jointsRawColor;
                            }
                            if (sj != null)
                            {
                                foreach (int j in joints)
                                {
                                    float x = sj[j].x;
                                    float y = sj[j].y;

                                    Rectangle jRect = new Rectangle((int)x, (int)y, jointSize, jointSize);

                                    if (Validation.ValidateRegion(jRect, boundsColor))
                                    {
                                        if (cType == ColorTypes.Bgra)
                                        {
                                            ((Image<Bgra, byte>)image).Draw(jRect, colorsBgra[i], -1);
                                        }
                                        else
                                        {
                                            ((Image<Bgr, byte>)image).Draw(jRect, colorsBgr[i], -1);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (cType == ColorTypes.Bgra)
                    {
                        return ((Image<Bgra, byte>)image).Bytes;
                    }
                    else
                    {
                        return ((Image<Bgr, byte>)image).Bytes;
                    }
                }

                /// <summary>Draws text above the Kinect skeleton onto the given BGRA image.</summary>
                public static byte[] TextToColor(byte[] image, KinectFrame frame, string text, bool flipLR, double textSize = 0.5)
                {
                    Bgra[] colorsBgra = { new Bgra(0, 0, 255, 255),
                                      new Bgra(0, 255, 0, 255),
                                      new Bgra(255, 0, 0, 255),
                                      new Bgra(255, 0, 255, 255),
                                      new Bgra(0, 255, 255, 255),
                                      new Bgra(255, 255, 0, 255) };

                    var ColorImage = Convert.BytesToImageBgra(image, frame.WidthColor, frame.HeightColor);
                    for (int i = 0; i < frame.MaxBodies; i++)
                    {
                        if (frame.BodyFrame[i].isTracked)
                        {
                            SkeletonJoint[] sj = frame.BodyFrame[i].skeletonData.jointsRawColor;
                            if (sj != null)
                            {
                                int x = (int)sj[3].x;
                                int y = (int)sj[3].y;

                                if (flipLR)
                                {
                                    x = frame.WidthColor - x;
                                }

                                CvInvoke.Rectangle(ColorImage, new Rectangle(x - 40, y - 47, 90, 25), colorsBgra[i].MCvScalar, -1);
                                CvInvoke.Rectangle(ColorImage, new Rectangle(x - 37, y - 44, 84, 19), new Bgra(255, 255, 255, 255).MCvScalar, -1);
                                CvInvoke.Rectangle(ColorImage, new Rectangle(x - 37, y - 44, 84, 19), new Bgra(0, 0, 0, 255).MCvScalar, 1);
                                CvInvoke.PutText(ColorImage, text + i, new Point(x - 30, y - 30), Emgu.CV.CvEnum.FontFace.HersheySimplex, textSize, new Bgra(0, 0, 0, 255).MCvScalar);
                            }
                        }
                    }

                    return ColorImage.Bytes;
                }

                /// <summary>Draws the Kinect face bounding box onto a color (BGRA or BGR) image.</summary>
                public static byte[] FaceToColor(KinectFrame frame, ColorTypes cType, int jointSize = 3)
                {
                    Bgra[] colorsBgra = { new Bgra(0, 0, 255, 255),
                                      new Bgra(0, 255, 0, 255),
                                      new Bgra(255, 0, 0, 255),
                                      new Bgra(255, 0, 255, 255),
                                      new Bgra(0, 255, 255, 255),
                                      new Bgra(255, 255, 0, 255) };

                    Bgr[] colorsBgr = { new Bgr(0, 0, 255),
                                    new Bgr(0, 255, 0),
                                    new Bgr(255, 0, 0),
                                    new Bgr(255, 0, 255),
                                    new Bgr(0, 255, 255),
                                    new Bgr(255, 255, 0) };

                    Rectangle boundsColor = new Rectangle(0, 0, frame.WidthColor - 1, frame.HeightColor - 1);

                    object image;
                    if (cType == ColorTypes.Bgra)
                    {
                        image = Convert.BytesToImageBgra(frame.ColorImage, frame.WidthColor, frame.HeightColor);
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        image = Convert.BytesToImageBgr(frame.ColorImage, frame.WidthColor, frame.HeightColor);
                    }
                    else
                    {
                        throw new Exception("cType must be Bgra or Bgr.");
                    }

                    for (int i = 0; i < frame.MaxBodies; i++)
                    {
                        if (frame.FaceBoundingBoxColor != null && frame.FaceBoundingBoxColor[i].Bottom != -1)
                        {
                            int top = frame.FaceBoundingBoxColor[i].Top;
                            int bottom = frame.FaceBoundingBoxColor[i].Bottom;
                            int left = frame.FaceBoundingBoxColor[i].Left;
                            int right = frame.FaceBoundingBoxColor[i].Right;

                            Rectangle faceRect = new Rectangle(left, top, right - left, bottom - top);

                            if (Validation.ValidateRegion(faceRect, boundsColor))
                            {
                                if (cType == ColorTypes.Bgra)
                                {
                                    ((Image<Bgra, byte>)image).Draw(faceRect, colorsBgra[i]);
                                }
                                else
                                {
                                    ((Image<Bgr, byte>)image).Draw(faceRect, colorsBgr[i]);
                                }
                            }
                        }
                    }

                    if (cType == ColorTypes.Bgra)
                    {
                        return ((Image<Bgra, byte>)image).Bytes;
                    }
                    else
                    {
                        return ((Image<Bgr, byte>)image).Bytes;
                    }
                }

                /// <summary>Flips the given image horizontally (left-right).</summary>
                public static byte[] FlipLR(byte[] data, int width, int height, ColorTypes cType)
                {
                    if (cType == ColorTypes.Bgra)
                    {
                        var result = Convert.BytesToImageBgra(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Horizontal).Bytes;
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        var result = Convert.BytesToImageBgr(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Horizontal).Bytes;
                    }
                    else if (cType == ColorTypes.Gray16)
                    {
                        var result = Convert.BytesToImageGray16(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Horizontal).Bytes;
                    }
                    else if (cType == ColorTypes.Gray8)
                    {
                        var result = Convert.BytesToImageGray8(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Horizontal).Bytes;
                    }
                    else
                    {
                        throw new Exception("ColorType must be one of Bgra, Bgr, Gray16 and Gray8.");
                    }
                }

                /// <summary>Flips the given image vertically (up-down).</summary>
                public static byte[] FlipUD(byte[] data, int width, int height, ColorTypes cType)
                {
                    if (cType == ColorTypes.Bgra)
                    {
                        var result = Convert.BytesToImageBgra(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Vertical).Bytes;
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        var result = Convert.BytesToImageBgr(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Vertical).Bytes;
                    }
                    else if (cType == ColorTypes.Gray16)
                    {
                        var result = Convert.BytesToImageGray16(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Vertical).Bytes;
                    }
                    else if (cType == ColorTypes.Gray8)
                    {
                        var result = Convert.BytesToImageGray8(data, width, height);

                        return result.Flip(Emgu.CV.CvEnum.FlipType.Vertical).Bytes;
                    }
                    else
                    {
                        throw new Exception("ColorType must be one of Bgra, Bgr, Gray16 and Gray8.");
                    }
                }

                /// <summary>Performs image scaling using the given factor and interpolation method.</summary>
                public static byte[] Scale(byte[] data, int width, int height, ColorTypes cType, float factor, Emgu.CV.CvEnum.Inter interpolationType)
                {
                    if (cType == ColorTypes.Bgra)
                    {
                        var result = Convert.BytesToImageBgra(data, width, height);

                        return result.Resize(factor, interpolationType).Bytes;
                    }
                    else if (cType == ColorTypes.Bgr)
                    {
                        var result = Convert.BytesToImageBgr(data, width, height);

                        return result.Resize(factor, interpolationType).Bytes;
                    }
                    else if (cType == ColorTypes.Gray16)
                    {
                        var result = Convert.BytesToImageGray16(data, width, height);

                        return result.Resize(factor, interpolationType).Bytes;
                    }
                    else if (cType == ColorTypes.Gray8)
                    {
                        var original = Convert.BytesToImageGray8(data, width, height);
                        var result = original.Resize(factor, interpolationType);

                        // fix scaling problem
                        if (result.Data.GetLength(1) > result.Width)
                        {
                            return null;
                        }
                        else if (result.Data.GetLength(1) < result.Width)
                        {
                            return null;
                        }
                        else
                        {
                            return result.Bytes;
                        }
                    }
                    else
                    {
                        throw new Exception("ColorType must be one of Bgra, Bgr, Gray16 and Gray8.");
                    }
                }

                /// <summary>Scales the given (BGRA) color image.</summary>
                public static byte[] ScaleColor(bool performScaling, byte[] img, int width, int height, float factor)
                {
                    if (performScaling)
                    {
                        try
                        {
                            return Scale(img, width, height, ColorTypes.Bgra, factor, Emgu.CV.CvEnum.Inter.Nearest);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.Message);

                            return img;
                        }
                    }

                    return img;
                }

                /// <summary>Scales the given gray16 image.</summary>
                public static byte[] ScaleGray16(bool performScaling, byte[] img, int width, int height, float factor)
                {
                    if (performScaling)
                    {
                        try
                        {
                            return Scale(img, width, height, ColorTypes.Gray16, factor, Emgu.CV.CvEnum.Inter.Nearest);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.Message);

                            return img;
                        }
                    }

                    return img;
                }

                /// <summary>Scales the given gray image.</summary>
                public static byte[] ScaleGray(bool performScaling, byte[] img, int width, int height, float factor)
                {
                    if (performScaling)
                    {
                        try
                        {
                            return Scale(img, width, height, ColorTypes.Gray8, factor, Emgu.CV.CvEnum.Inter.Nearest);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.Message);

                            return img;
                        }
                    }

                    return img;
                }

                /// <summary>Scales the coordinates of the given body frames.</summary>
                public static DataFrames.Kinect.Body[] ScaleBodyFrame(DataFrames.Kinect.Body[] b, bool performScalingColor, float factorColor, int widthColor, int heightColor, bool performScalingGray, float factorGray, int widthGray, int heightGray)
                {
                    DataFrames.Kinect.Body[] result = new DataFrames.Kinect.Body[b.Length];

                    for (int i = 0; i < b.Length; i++)
                    {
                        result[i].firstTrack = b[i].firstTrack;
                        result[i].isTracked = b[i].isTracked;
                        result[i].leanConfidence = b[i].leanConfidence;
                        result[i].leanFB = b[i].leanFB;
                        result[i].leanLR = b[i].leanLR;

                        result[i].skeletonData = ScaleSkeleton(b[i].skeletonData, performScalingColor, factorColor, widthColor, heightColor, performScalingGray, factorGray, widthGray, heightGray);
                    }

                    return result;
                }

                /// <summary>Scales the coordinates of the given skeleton.</summary>
                private static Skeleton ScaleSkeleton(Skeleton sk, bool performScalingColor, float factorColor, int widthColor, int heightColor, bool performScalingGray, float factorGray, int widthGray, int heightGray)
                {
                    Skeleton result = new Skeleton();

                    result.jointsRaw = sk.jointsRaw;
                    result.jointsFiltered = sk.jointsFiltered;

                    if (performScalingColor && sk.jointsRawColor != null && sk.jointsFilteredColor != null)
                    {
                        Rectangle boundsColor = new Rectangle(0, 0, widthColor - 1, heightColor - 1);
                        try
                        {
                            result.jointsRawColor = FrameOperations.Transform.ScaleJoints(sk.jointsRawColor, factorColor, boundsColor);
                            result.jointsFilteredColor = FrameOperations.Transform.ScaleJoints(sk.jointsFilteredColor, factorColor, boundsColor);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.Message);

                            result.jointsRawColor = null;
                            result.jointsFilteredColor = null;
                        }
                    }
                    else
                    {
                        result.jointsRawColor = sk.jointsRawColor;
                        result.jointsFilteredColor = sk.jointsFilteredColor;
                    }

                    if (performScalingGray && sk.jointsRawGray != null && sk.jointsFilteredGray != null)
                    {
                        Rectangle boundsGray = new Rectangle(0, 0, widthGray - 1, heightGray - 1);
                        try
                        {
                            result.jointsRawGray = FrameOperations.Transform.ScaleJoints(sk.jointsRawGray, factorGray, boundsGray);
                            result.jointsFilteredGray = FrameOperations.Transform.ScaleJoints(sk.jointsFilteredGray, factorGray, boundsGray);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.Message);

                            result.jointsRawGray = null;
                            result.jointsFilteredGray = null;
                        }
                    }
                    else
                    {
                        result.jointsRawGray = sk.jointsRawGray;
                        result.jointsFilteredGray = sk.jointsFilteredGray;
                    }

                    return result;
                }

                /// <summary>Scales the coordinates of the given joints with respect to the given bounds.</summary>
                private static SkeletonJoint[] ScaleJoints(SkeletonJoint[] joints, float factor, Rectangle bounds)
                {
                    if (joints == null) { return null; }

                    SkeletonJoint[] result = new SkeletonJoint[joints.Length];
                    for (int i = 0; i < joints.Length; i++)
                    {
                        int newX = (int)(joints[i].x * factor);
                        int newY = (int)(joints[i].y * factor);

                        if (Validation.ValidateRegion(new Rectangle(newX, newY, 1, 1), bounds))
                        {
                            result[i].x = newX;
                            result[i].y = newY;
                        }
                        else
                        {
                            result[i].x = 0;
                            result[i].y = 0;
                        }

                        result[i].z = joints[i].z;
                        result[i].confidence = joints[i].confidence;
                    }

                    return result;
                }

                /// <summary>Scales the coordinates of the given face bounding boxes with respect to the given bounds for width and height.</summary>
                public static FaceBox[] ScaleFaceBox(FaceBox[] facebox, bool performScaling, float factor, int width, int height)
                {
                    if (facebox == null) { return null; }

                    FaceBox[] result = new FaceBox[facebox.Length];

                    if (performScaling)
                    {
                        Rectangle bounds = new Rectangle(0, 0, width, height);
                        for (int i = 0; i < facebox.Length; i++)
                        {
                            FaceBox fb = facebox[i];
                            if (Validation.ValidateRegion(new Rectangle(fb.Left, fb.Top, fb.Right - fb.Left, fb.Bottom - fb.Top), bounds))
                            {
                                result[i].Bottom = (int)(fb.Bottom * factor);
                                result[i].Left = (int)(fb.Left * factor);
                                result[i].Right = (int)(fb.Right * factor);
                                result[i].Top = (int)(fb.Top * factor);
                            }
                            else
                            {
                                result[i].Bottom = -1;
                                result[i].Left = -1;
                                result[i].Right = -1;
                                result[i].Top = -1;
                            }
                        }
                    }
                    else
                    {
                        result = facebox;
                    }

                    return result;
                }

            } // end class

        } // end class
    }

}