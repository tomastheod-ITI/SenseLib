
////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                            ////
////   Copyright (c) 2016 - present, CERTH-ITI, Artica                                          ////
////                                                                                            ////
////////////////////////////////////////////////////////////////////////////////////////////////////


using Microsoft.Band;
using Microsoft.Band.Sensors;
using SenseLib.DataFrames.MSBand;
using System;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Threading.Tasks;


namespace SenseLib
{

    namespace DataFrames
    {

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

        namespace MSBand
        {
            public sealed class BandSession
            {
                private static BandSession instance = null;

                public IBandInfo[] bandInfo { get; set; }

                public IBandClient[] bandClient { get; set; }

                public string[] bandName { get; set; }

                public static BandSession Instance
                {
                    get
                    {
                        if (instance == null)
                        {
                            instance = new BandSession();
                        }

                        return instance;
                    }
                }


                private BandSession() { }

            }

            public struct BandSelection
            {
                public bool heartRate;
                public bool skinTemperature;
                public bool pedometer;
                public bool calories;
                public bool distance;
                public bool accelerometer;
                public bool gyroscope;
                public bool gsr;

                public BandSelection(bool init)
                {
                    heartRate = init;
                    skinTemperature = init;
                    accelerometer = init;
                    calories = init;
                    gyroscope = init;
                    pedometer = init;
                    distance = init;
                    gsr = init;
                }

                public BandSelection(bool[] init)
                {
                    heartRate = init[0];
                    skinTemperature = init[1];
                    accelerometer = init[2];
                    calories = init[3];
                    gyroscope = init[4];
                    pedometer = init[5];
                    distance = init[6];
                    gsr = init[7];
                }
            }

            public class MSBandSenseLib
            {
                private const int sleepTime = 3;

                private const int maxIdleTime = 3 * 60 * 1000;

                /// <summary> Indicates whether connection to the band has been lost. </summary>
                public bool IsIdle { get; private set; }

                public IBandClient bandClient { get; set; }

                public string bandName { get; private set; }

                public BandSelection bandSelection { get; set; }

                // Local data
                private HeartRateDataBand heartRateData;
                private SkinTemperatureDataBand skinTemperatureData;
                private PedometerDataBand pedometerData;
                private CaloriesDataBand caloriesData;
                private DistanceDataBand distanceData;
                private GyroscopeDataBand gyroscopeData;
                private GalvanicSkinResponse galvanicData;

                private readonly object myLock;
                private MSBandFrame bandFrame;

                private DateTime[] sendTime;


                public MSBandSenseLib(IBandClient vBandClient, BandSelection vBandSelection, string vBandName)
                {
                    bandClient = vBandClient;
                    bandSelection = vBandSelection;
                    bandName = vBandName;

                    IsIdle = false;

                    myLock = new object();
                    bandFrame = new MSBandFrame();

                    bandFrame.SensorID = vBandName;

                    sendTime = new DateTime[2];
                }

                public void UpdateBandAcquisition(IBandClient vBandClient, string vBandName)
                {
                    bandClient = vBandClient;
                    bandName = vBandName;

                    bandFrame.SensorID = vBandName;

                    try
                    {
                        OpenSensor();
                    }
                    catch (Exception)
                    {

                    }
                }

                public bool IsSensorAvailable(int msTimeout)
                {
                    if (bandClient == null)
                        return false;

                    return true;
                }

                public void OpenSensor()
                {
                    p_readHeartReate();
                    p_readingSkinTemperature();
                    p_readingPedometer();
                    p_readingCalories();
                    p_readingDistance();
                    p_readingGyroscope();
                    p_readingGalvanic();
                }

                public void CloseSensor()
                {
                    p_stopReading();
                }

                public MSBandFrame GetFrame()
                {
                    DateTime totalTime = DateTime.Now;
                    while (sendTime[0] == sendTime[1])
                    {
                        Task.Delay(sleepTime).Wait();

                        if (DateTime.Now.AddTicks(-totalTime.Ticks).Ticks / 10000.0 > maxIdleTime)
                        {
                            IsIdle = true;

                            return null;
                        }
                    }
                    Task.Delay(1).Wait();

                    IsIdle = false;
                    sendTime[1] = sendTime[0];

                    lock (myLock)
                    {
                        bandFrame.TimeStamp = sendTime[0];

                        bandFrame.HeartRateData = heartRateData;
                        bandFrame.SkinTemperatureData = skinTemperatureData;
                        bandFrame.PedometerData = pedometerData;
                        bandFrame.CaloriesData = caloriesData;
                        bandFrame.DistanceData = distanceData;
                        bandFrame.GyroscopeData = gyroscopeData;
                        bandFrame.GalvanicSkinResponseData = galvanicData;

                        return bandFrame;
                    }
                }

                public byte[] FrameToBytes(MSBandFrame frame)
                {
                    lock (myLock)
                    {
                        return Utils.Serialization.SerializeXML(frame);
                    }
                }

                public MSBandFrame BytesToFrame(byte[] frameBytes)
                {
                    return Utils.Serialization.DeserializeXML<MSBandFrame>(frameBytes);
                }

                #region Start / Stop reading

                public async void p_readHeartReate()
                {
                    bandClient.SensorManager.HeartRate.ReadingChanged += HeartRate_ReadingChanged;
                    await bandClient.SensorManager.HeartRate.StartReadingsAsync();
                }

                public async void p_readingSkinTemperature()
                {
                    bandClient.SensorManager.SkinTemperature.ReadingChanged += SkinTemperature_ReadingChanged;
                    await bandClient.SensorManager.SkinTemperature.StartReadingsAsync();
                }

                public async void p_readingPedometer()
                {
                    bandClient.SensorManager.Pedometer.ReadingChanged += Pedometer_ReadingChanged;
                    await bandClient.SensorManager.Pedometer.StartReadingsAsync();
                }

                public async void p_readingCalories()
                {
                    bandClient.SensorManager.Calories.ReadingChanged += Calories_ReadingChanged;
                    await bandClient.SensorManager.Calories.StartReadingsAsync();
                }

                public async void p_readingDistance()
                {
                    bandClient.SensorManager.Distance.ReadingChanged += Distance_ReadingChanged;
                    await bandClient.SensorManager.Distance.StartReadingsAsync();
                }

                public async void p_readingGyroscope()
                {
                    bandClient.SensorManager.Gyroscope.ReadingChanged += Gyroscope_ReadingChanged;
                    await bandClient.SensorManager.Gyroscope.StartReadingsAsync();
                }

                public async void p_readingGalvanic()
                {
                    bandClient.SensorManager.Gsr.ReadingChanged += Gsr_ReadingChanged;
                    await bandClient.SensorManager.Gsr.StartReadingsAsync();
                }

                private async void p_stopReading()
                {
                    await bandClient.SensorManager.HeartRate.StopReadingsAsync();
                    await bandClient.SensorManager.SkinTemperature.StopReadingsAsync();
                    await bandClient.SensorManager.Pedometer.StopReadingsAsync();
                    await bandClient.SensorManager.Calories.StopReadingsAsync();
                    await bandClient.SensorManager.Distance.StopReadingsAsync();
                    await bandClient.SensorManager.Accelerometer.StopReadingsAsync();
                    await bandClient.SensorManager.Gyroscope.StopReadingsAsync();
                    await bandClient.SensorManager.Gsr.StopReadingsAsync();
                }

                #endregion

                #region Readers

                private void HeartRate_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandHeartRateReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        heartRateData.Data = e.SensorReading.HeartRate;
                        heartRateData.DataTimeStamp = sendTime[0];
                    }
                }

                private void SkinTemperature_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandSkinTemperatureReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        skinTemperatureData.Data = e.SensorReading.Temperature;
                        skinTemperatureData.DataTimeStamp = sendTime[0];
                    }
                }

                private void Pedometer_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandPedometerReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        pedometerData.StepsToday = e.SensorReading.StepsToday;
                        pedometerData.StepsTotal = e.SensorReading.TotalSteps;
                        pedometerData.DataTimeStamp = sendTime[0];
                    }
                }

                private void Calories_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandCaloriesReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        caloriesData.DataTotal = e.SensorReading.Calories;
                        caloriesData.DataToday = e.SensorReading.CaloriesToday;
                        caloriesData.DataTimeStamp = sendTime[0];
                    }
                }

                private void Distance_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandDistanceReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        distanceData.DistanteToday = e.SensorReading.DistanceToday;
                        distanceData.Pace = e.SensorReading.Pace;
                        distanceData.Speed = e.SensorReading.Speed;
                        distanceData.DataTimeStamp = sendTime[0];
                    }
                }

                private void Gyroscope_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandGyroscopeReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        gyroscopeData.AccelerationX = e.SensorReading.AccelerationX;
                        gyroscopeData.AccelerationY = e.SensorReading.AccelerationY;
                        gyroscopeData.AccelerationZ = e.SensorReading.AccelerationZ;

                        gyroscopeData.AngularVelocityX = e.SensorReading.AngularVelocityX;
                        gyroscopeData.AngularVelocityY = e.SensorReading.AngularVelocityY;
                        gyroscopeData.AngularVelocityZ = e.SensorReading.AngularVelocityZ;

                        gyroscopeData.DataTimeStamp = sendTime[0];
                    }
                }

                private void Gsr_ReadingChanged(object sender, BandSensorReadingEventArgs<IBandGsrReading> e)
                {
                    lock (myLock)
                    {
                        sendTime[0] = DateTime.Now;

                        galvanicData.Gsr = e.SensorReading.Resistance;
                        galvanicData.DataTimeStamp = sendTime[0];
                    }
                }

                #endregion

            }
        }

    }

    namespace Utils
    {
        /// <summary>Provides methods for Serializing arbitrary objects using an XML format.</summary>
        public static class Serialization
        {
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
        }
    }
   
}