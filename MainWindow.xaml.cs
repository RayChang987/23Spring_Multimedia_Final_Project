//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using MQTTnet;
    using MQTTnet.Client;
    using MQTTnet.Client.Options;
    using System.Threading.Tasks;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;
        private DrawingGroup drawingGroup1;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        private DrawingImage imageSource1;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;





        /// <summary>
        /// Number of samples captured from Kinect audio stream each millisecond.
        /// </summary>
        private const int SamplesPerMillisecond = 16;

        /// <summary>
        /// Number of bytes in each Kinect audio stream sample (32-bit IEEE float).
        /// </summary>
        private const int BytesPerSample = sizeof(float);

        /// <summary>
        /// Number of audio samples represented by each column of pixels in wave bitmap.
        /// </summary>
        private const int SamplesPerColumn = 40;

        /// <summary>
        /// Minimum energy of audio to display (a negative number in dB value, where 0 dB is full scale)
        /// </summary>
        private const int MinEnergy = -90;

        /// <summary>
        /// Width of bitmap that stores audio stream energy data ready for visualization.
        /// </summary>
        private const int EnergyBitmapWidth = 780;

        /// <summary>
        /// Height of bitmap that stores audio stream energy data ready for visualization.
        /// </summary>
        private const int EnergyBitmapHeight = 195;

        /// <summary>
        /// Bitmap that contains constructed visualization for audio stream energy, ready to
        /// be displayed. It is a 2-color bitmap with white as background color and blue as
        /// foreground color.
        /// </summary>
        private readonly WriteableBitmap energyBitmap;

        /// <summary>
        /// Rectangle representing the entire energy bitmap area. Used when drawing background
        /// for energy visualization.
        /// </summary>
        private readonly Int32Rect fullEnergyRect = new Int32Rect(0, 0, EnergyBitmapWidth, EnergyBitmapHeight);

        /// <summary>
        /// Array of background-color pixels corresponding to an area equal to the size of whole energy bitmap.
        /// </summary>
        private readonly byte[] backgroundPixels = new byte[EnergyBitmapWidth * EnergyBitmapHeight];

        /// <summary>
        /// Will be allocated a buffer to hold a single sub frame of audio data read from audio stream.
        /// </summary>
        private readonly byte[] audioBuffer = null;

        /// <summary>
        /// Buffer used to store audio stream energy data as we read audio.
        /// We store 25% more energy values than we strictly need for visualization to allow for a smoother
        /// stream animation effect, since rendering happens on a different schedule with respect to audio
        /// capture.
        /// </summary>
        private readonly float[] energy = new float[(uint)(EnergyBitmapWidth * 1.25)];

        /// <summary>
        /// Object for locking energy buffer to synchronize threads.
        /// </summary>
        private readonly object energyLock = new object();

        /// <summary>
        /// Reader for audio frames
        /// </summary>
        private AudioBeamFrameReader reader = null;

        /// <summary>
        /// Last observed audio beam angle in radians, in the range [-pi/2, +pi/2]
        /// </summary>
        private float beamAngle = 0;

        /// <summary>
        /// Last observed audio beam angle confidence, in the range [0, 1]
        /// </summary>
        private float beamAngleConfidence = 0;

        /// <summary>
        /// Array of foreground-color pixels corresponding to a line as long as the energy bitmap is tall.
        /// This gets re-used while constructing the energy visualization.
        /// </summary>
        private byte[] foregroundPixels;

        /// <summary>
        /// Sum of squares of audio samples being accumulated to compute the next energy value.
        /// </summary>
        private float accumulatedSquareSum;

        /// <summary>
        /// Number of audio samples accumulated so far to compute the next energy value.
        /// </summary>
        private int accumulatedSampleCount;

        /// <summary>
        /// Index of next element available in audio energy buffer.
        /// </summary>
        private int energyIndex;

        /// <summary>
        /// Number of newly calculated audio stream energy values that have not yet been
        /// displayed.
        /// </summary>
        private int newEnergyAvailable;

        /// <summary>
        /// Error between time slice we wanted to display and time slice that we ended up
        /// displaying, given that we have to display in integer pixels.
        /// </summary>
        private float energyError;

        /// <summary>
        /// Last time energy visualization was rendered to screen.
        /// </summary>
        private DateTime? lastEnergyRefreshTime;

        /// <summary>
        /// Index of first energy element that has never (yet) been displayed to screen.
        /// </summary>
        private int energyRefreshIndex;

        /// <summary>
        /// Create the MQTT client object.
        /// </summary>
        IMqttClient client;

        /// <summary>
        /// Record the last send signal.
        /// </summary>
        int last_signal0 = -1;
        int last_signal1 = -1;

        private string carText1 = "Car1 : None";

        private string carText2 = "Car2 : None";

        ActionCode actionCode0 = ActionCode.None;
        ActionCode actionCode1 = ActionCode.None;

        DateTime? lastStopTime0;
        DateTime? lastStopTime1;

        enum ActionCode
        {
            None = 0,
            Left = 1,
            Right = 2,
            Forward = 3,
            Back = 4,
            Stop = 5,
            GameStart = 6,
            Error = 7
        }

        private Tuple<ActionCode, ActionCode> get_control_signal()
        {
            if (this.beamAngleConfidence > 0.9f)
            {
                int baseIndex = (this.energyRefreshIndex + this.energy.Length - EnergyBitmapWidth) % this.energy.Length;
                if (this.energy[(baseIndex) % this.energy.Length] > 10f)
                {//0.3
                    if (this.beamAngle < 0)
                    {
                        this.actionCode0 = ActionCode.Stop;
                        lastStopTime0 = DateTime.UtcNow;
                    }
                    else
                    {
                        this.actionCode1 = ActionCode.Stop;
                        lastStopTime1 = DateTime.UtcNow;
                    }
                }
            }

            if (lastStopTime0 != null)
            {
                if ((DateTime.UtcNow - (DateTime)lastStopTime0).Seconds < 2)
                {
                    this.actionCode0 = ActionCode.Stop;
                }
            }

            if (lastStopTime1 != null)
            {
                if ((DateTime.UtcNow - (DateTime)lastStopTime1).Seconds < 2)
                {
                    this.actionCode1 = ActionCode.Stop;
                }
            }



            this.car1message.Content = "Car1 : " + actionCode0.ToString();
            this.car2message.Content = "Car2 : " + actionCode1.ToString();

            return new Tuple<ActionCode, ActionCode>(this.actionCode0, this.actionCode1);
        }

        private void send_sigals(Tuple<ActionCode, ActionCode> actionCodes)
        {
            this.signal_info.Content = actionCodes.Item1.ToString() + " " + actionCodes.Item2.ToString();

            if ((int)actionCodes.Item1 != last_signal0)
            {
                last_signal0 = (int)actionCodes.Item1;
                PublishMessage("car0", (int)actionCodes.Item1);
            }
            if ((int)actionCodes.Item2 != last_signal1)
            {
                last_signal1 = (int)actionCodes.Item2;
                PublishMessage("car1", (int)actionCodes.Item2);
            }

        }

        /// <summary>
        /// Establish connection.
        /// </summary>
        /// <param name="ip">ip that we need to connect</param>
        private void EstablishConnect(string ip)
        {

            var mqttFactory = new MqttFactory();
            this.client = mqttFactory.CreateMqttClient();

            var options = new MqttClientOptionsBuilder()
                         .WithClientId(Guid.NewGuid().ToString())
                         .WithTcpServer(ip, 1883)
                         .WithCleanSession()
                         .Build();

            client.UseDisconnectedHandler(e =>
            {
                MessageBox.Show("Connected to the broker unsuccessfully");
            });

            client.ConnectAsync(options);

        }

        /// <summary>
        /// Publish message.
        /// </summary>
        /// <param name="topic">we need to publish the message to this topic</param>
        /// <param name="s">the message that we need to publish</param>
        private void PublishMessage(string topic, int s)
        {

            var message = new MqttApplicationMessageBuilder()
                            .WithTopic(topic)
                            .WithPayload(s.ToString())
                            .WithAtLeastOnceQoS()
                            .Build();

            if (client.IsConnected)
            {
                client.PublishAsync(message);
            }
            else
            {
                MessageBox.Show("Client doesn't connect to MQTTserver");
            }
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();





            // Get its audio source
            AudioSource audioSource = this.kinectSensor.AudioSource;

            // Allocate 1024 bytes to hold a single audio sub frame. Duration sub frame 
            // is 16 msec, the sample rate is 16khz, which means 256 samples per sub frame. 
            // With 4 bytes per sample, that gives us 1024 bytes.
            this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];

            // Open the reader for the audio frames
            this.reader = audioSource.OpenReader();








            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
            this.drawingGroup1 = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.imageSource1 = new DrawingImage(this.drawingGroup1);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            //Connect to the MQTTserver
            EstablishConnect("122.116.34.215");

            // initialize the components (controls) of the window
            this.InitializeComponent();

            label_info.Content = "a";

            this.car1view.Navigate("https://www.google.com");
            this.car2view.Navigate("https://www.google.com");
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }
        public ImageSource ImageSource1
        {
            get
            {
                return this.imageSource1;
            }
        }

        public string CarText1
        {
            get
            {
                return this.carText1;
            }
        }

        public string CarText2
        {
            get
            {
                return this.carText2;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }


            // Initialize foreground pixels
            this.foregroundPixels = new byte[EnergyBitmapHeight];
            for (int i = 0; i < this.foregroundPixels.Length; ++i)
            {
                this.foregroundPixels[i] = 0xff;
            }

            CompositionTarget.Rendering += this.UpdateEnergy;

            if (this.reader != null)
            {
                // Subscribe to new audio frame arrived events
                this.reader.FrameArrived += this.Reader_Audio_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            CompositionTarget.Rendering -= this.UpdateEnergy;

            if (this.reader != null)
            {
                // AudioBeamFrameReader is IDisposable
                this.reader.Dispose();
                this.reader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                List<Body> bodys = new List<Body>();
                if (this.bodies.Length == 0) return;
                Body leftBody = this.bodies[0];
                Body rightBody = this.bodies[0];
                bool hasLeft = false;
                bool hasRight = false;
                foreach (var b in this.bodies)
                {
                    if (b.IsTracked)
                        bodys.Add(b);
                }
                foreach (var b in bodys)
                {
                    IReadOnlyDictionary<JointType, Joint> joints = b.Joints;

                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                    foreach (JointType jointType in joints.Keys)
                    {
                        CameraSpacePoint position = joints[jointType].Position;
                        if (position.Z < 0)
                        {
                            position.Z = InferredZPositionClamp;
                        }

                        DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                        jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                    }

                    if (jointPoints[JointType.SpineShoulder].X <= 275)
                    {
                        hasLeft = true;
                        leftBody = b;
                    }
                    else if (jointPoints[JointType.SpineShoulder].X > 275)
                    {
                        hasRight = true;
                        rightBody = b;
                    }
                }

                string sout = "";


                string str = "result: ";

                str += "hasLeft " + hasLeft.ToString() + ", hasRight " + hasRight.ToString();

                if (hasLeft)
                {
                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        // Draw a transparent background to set the render size
                        dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                        int penIndex = 0;

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (bodys.Count > 0)
                        {

                            Body body = leftBody;

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                                sout += jointType + " " + depthSpacePoint.X + " " + depthSpacePoint.Y + "\n";
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);


                            sout += "result:";
                            /*
                            if (body.HandLeftState == HandState.Open && body.HandRightState == HandState.Open)
                            {
                                actionCode0 = ActionCode.Forward;
                            }
                            if (body.HandLeftState == HandState.Open && body.HandRightState == HandState.Closed)
                            {
                                actionCode0 = ActionCode.Right;
                            }
                            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Open)
                            {
                                actionCode0 = ActionCode.Left;
                            }
                            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Closed)
                            {
                                actionCode0 = ActionCode.Back;
                            }
                            if (body.HandLeftState == HandState.Lasso || body.HandRightState == HandState.Lasso)
                            {
                                actionCode0 = ActionCode.GameStart;
                            }*/
                            if (Math.Abs(jointPoints[JointType.HandLeft].Y - jointPoints[JointType.HandRight].Y) < 30)
                            {
                                actionCode0 = ActionCode.Forward;
                            }
                            else if (jointPoints[JointType.HandLeft].Y > jointPoints[JointType.HandRight].Y)
                            {
                                actionCode0 = ActionCode.Right;
                            }
                            else if (jointPoints[JointType.HandLeft].Y < jointPoints[JointType.HandRight].Y)
                            {
                                actionCode0 = ActionCode.Left;
                            }
                            if (body.HandLeftState == HandState.Open && body.HandRightState == HandState.Open)
                            {
                                actionCode0 = ActionCode.Back;
                            }
                            label_info.Content = sout;
                        }

                        // prevent drawing outside of our render area
                        this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                }


                sout += "\n\n==========================\n\n";

                if (hasRight)
                {
                    using (DrawingContext dc = this.drawingGroup1.Open())
                    {
                        // Draw a transparent background to set the render size
                        dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                        int penIndex = 1;

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (bodys.Count > 0)
                        {

                            Body body = rightBody;

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                                sout += jointType + " " + depthSpacePoint.X + " " + depthSpacePoint.Y + "\n";
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);


                            sout += "result:";
                            /*
                            if (body.HandLeftState == HandState.Open && body.HandRightState == HandState.Open)
                            {
                                actionCode1 = ActionCode.Forward;
                            }
                            if (body.HandLeftState == HandState.Open && body.HandRightState == HandState.Closed)
                            {
                                actionCode1 = ActionCode.Right;
                            }
                            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Open)
                            {
                                actionCode1 = ActionCode.Left;
                            }
                            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Closed)
                            {
                                actionCode1 = ActionCode.Back;
                            }
                            if (body.HandLeftState == HandState.Lasso || body.HandRightState == HandState.Lasso)
                            {
                                actionCode1 = ActionCode.GameStart;
                            }
                            */

                            if (Math.Abs(jointPoints[JointType.HandLeft].Y - jointPoints[JointType.HandRight].Y) < 30)
                            {
                                actionCode1 = ActionCode.Forward;
                            }
                            else if (jointPoints[JointType.HandLeft].Y > jointPoints[JointType.HandRight].Y)
                            {
                                actionCode1 = ActionCode.Right;
                            }
                            else if (jointPoints[JointType.HandLeft].Y < jointPoints[JointType.HandRight].Y)
                            {
                                actionCode1 = ActionCode.Left;
                            }
                            if (body.HandLeftState == HandState.Open && body.HandRightState == HandState.Open)
                            {
                                actionCode1 = ActionCode.Back;
                            }

                            label_info.Content = sout;
                        }



                        // prevent drawing outside of our render area
                        this.drawingGroup1.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                }
                str += "body count : " + this.bodies.Length;
                this.label_info.Content = str;
                this.action_info.Content = sout;//fuck 打錯字
                //run run run run run run 
            }

            send_sigals(this.get_control_signal());
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }






        /// <summary>
        /// Handles the audio frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_Audio_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            AudioBeamFrameReference frameReference = e.FrameReference;
            AudioBeamFrameList frameList = frameReference.AcquireBeamFrames();

            if (frameList != null)
            {
                // AudioBeamFrameList is IDisposable
                using (frameList)
                {
                    // Only one audio beam is supported. Get the sub frame list for this beam
                    IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                    // Loop over all sub frames, extract audio buffer and beam information
                    foreach (AudioBeamSubFrame subFrame in subFrameList)
                    {
                        // Check if beam angle and/or confidence have changed
                        bool updateBeam = false;

                        if (subFrame.BeamAngle != this.beamAngle)
                        {
                            this.beamAngle = subFrame.BeamAngle;
                            updateBeam = true;
                        }

                        if (subFrame.BeamAngleConfidence != this.beamAngleConfidence)
                        {
                            this.beamAngleConfidence = subFrame.BeamAngleConfidence;
                            updateBeam = true;
                        }

                        if (updateBeam)
                        {
                            // Refresh display of audio beam
                            this.AudioBeamChanged();
                        }

                        // Process audio buffer
                        subFrame.CopyFrameDataToArray(this.audioBuffer);

                        for (int i = 0; i < this.audioBuffer.Length; i += BytesPerSample)
                        {
                            // Extract the 32-bit IEEE float sample from the byte array
                            float audioSample = BitConverter.ToSingle(this.audioBuffer, i);

                            this.accumulatedSquareSum += audioSample * audioSample;
                            ++this.accumulatedSampleCount;

                            if (this.accumulatedSampleCount < SamplesPerColumn)
                            {
                                continue;
                            }

                            float meanSquare = this.accumulatedSquareSum / SamplesPerColumn;

                            if (meanSquare > 1.0f)
                            {
                                // A loud audio source right next to the sensor may result in mean square values
                                // greater than 1.0. Cap it at 1.0f for display purposes.
                                meanSquare = 1.0f;
                            }

                            // Calculate energy in dB, in the range [MinEnergy, 0], where MinEnergy < 0
                            float energy = MinEnergy;

                            if (meanSquare > 0)
                            {
                                energy = (float)(10.0 * Math.Log10(meanSquare));
                            }

                            lock (this.energyLock)
                            {
                                // Normalize values to the range [0, 1] for display
                                this.energy[this.energyIndex] = (MinEnergy - energy) / MinEnergy;
                                this.energyIndex = (this.energyIndex + 1) % this.energy.Length;
                                ++this.newEnergyAvailable;
                            }

                            this.accumulatedSquareSum = 0;
                            this.accumulatedSampleCount = 0;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Method called when audio beam angle and/or confidence have changed.
        /// </summary>
        private void AudioBeamChanged()
        {
            // Maximum possible confidence corresponds to this gradient width
            const float MinGradientWidth = 0.04f;

            // Set width of mark based on confidence.
            // A confidence of 0 would give us a gradient that fills whole area diffusely.
            // A confidence of 1 would give us the narrowest allowed gradient width.
            float halfWidth = Math.Max(1 - this.beamAngleConfidence, MinGradientWidth) / 2;

            // Update the gradient representing sound source position to reflect confidence
            // this.beamBarGsPre.Offset = Math.Max(this.beamBarGsMain.Offset - halfWidth, 0);
            // this.beamBarGsPost.Offset = Math.Min(this.beamBarGsMain.Offset + halfWidth, 1);

            // Convert from radians to degrees for display purposes
            float beamAngleInDeg = this.beamAngle * 180.0f / (float)Math.PI;

            // Rotate gradient to match angle
            // beamBarRotation.Angle = -beamAngleInDeg;
            // beamNeedleRotation.Angle = -beamAngleInDeg;

            int baseIndex = (this.energyRefreshIndex + this.energy.Length - EnergyBitmapWidth) % this.energy.Length;


            this.audio_info.Content = beamAngleInDeg.ToString() + " | " + this.beamAngleConfidence.ToString() + "\n" + this.energy[(baseIndex) % this.energy.Length].ToString();

            // Display new numerical values
            // beamAngleText.Text = string.Format(CultureInfo.CurrentCulture, Properties.Resources.BeamAngle, beamAngleInDeg.ToString("0", CultureInfo.CurrentCulture));
            // beamConfidenceText.Text = string.Format(CultureInfo.CurrentCulture, Properties.Resources.BeamAngleConfidence, this.beamAngleConfidence.ToString("0.00", CultureInfo.CurrentCulture));
        }

        /// <summary>
        /// Handles rendering energy visualization into a bitmap.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void UpdateEnergy(object sender, EventArgs e)
        {
            lock (this.energyLock)
            {
                // Calculate how many energy samples we need to advance since the last update in order to
                // have a smooth animation effect
                DateTime now = DateTime.UtcNow;
                DateTime? previousRefreshTime = this.lastEnergyRefreshTime;
                this.lastEnergyRefreshTime = now;

                // No need to refresh if there is no new energy available to render
                if (this.newEnergyAvailable <= 0)
                {
                    return;
                }

                if (previousRefreshTime != null)
                {
                    float energyToAdvance = this.energyError + (((float)(now - previousRefreshTime.Value).TotalMilliseconds * SamplesPerMillisecond) / SamplesPerColumn);
                    int energySamplesToAdvance = Math.Min(this.newEnergyAvailable, (int)Math.Round(energyToAdvance));
                    this.energyError = energyToAdvance - energySamplesToAdvance;
                    this.energyRefreshIndex = (this.energyRefreshIndex + energySamplesToAdvance) % this.energy.Length;
                    this.newEnergyAvailable -= energySamplesToAdvance;
                }

                // // // clear background of energy visualization area
                // // this.energyBitmap.WritePixels(this.fullEnergyRect, this.backgroundPixels, EnergyBitmapWidth, 0);

                // // Draw each energy sample as a centered vertical bar, where the length of each bar is
                // // proportional to the amount of energy it represents.
                // // Time advances from left to right, with current time represented by the rightmost bar.
                // int baseIndex = (this.energyRefreshIndex + this.energy.Length - EnergyBitmapWidth) % this.energy.Length;
                // for (int i = 0; i < EnergyBitmapWidth; ++i)
                // {
                //     const int HalfImageHeight = EnergyBitmapHeight / 2;

                //     // Each bar has a minimum height of 1 (to get a steady signal down the middle) and a maximum height
                //     // equal to the bitmap height.
                //     int barHeight = (int)Math.Max(1.0, this.energy[(baseIndex + i) % this.energy.Length] * EnergyBitmapHeight);

                //     // Center bar vertically on image
                //     var barRect = new Int32Rect(i, HalfImageHeight - (barHeight / 2), 1, barHeight);

                //     // Draw bar in foreground color
                //     this.energyBitmap.WritePixels(barRect, this.foregroundPixels, 1, 0);
                // }
            }
        }












        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
