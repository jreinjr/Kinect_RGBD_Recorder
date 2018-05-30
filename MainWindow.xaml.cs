using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.IO;

namespace Kinect_RGBD_Recorder
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        const int MapDepthToByte = 8000 / 256;
        KinectSensor ks;
        MultiSourceFrameReader multiSourceFrameReader;
        CoordinateMapper coordinateMapper;
        byte[] colorPixels;
        byte[] depthPixels;
        ColorImageFormat colorFormat;
        ColorImageFormat depthFormat;
        WriteableBitmap writeableBitmap;
        BitmapSource colorBitmapSource;
        BitmapSource depthBitmapSource;
        int imageSerial;
        bool recordStarted;
        DepthSpacePoint[] colorMappedToDepthPoints;
        FrameDescription colorFrameDesc;
        FrameDescription depthFrameDesc;

        public MainWindow()
        {
            InitializeComponent();

            ks = KinectSensor.GetDefault();
            ks.Open();

            coordinateMapper = ks.CoordinateMapper;

            colorFrameDesc = ks.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            int colorWidth = colorFrameDesc.Width;
            int colorHeight = colorFrameDesc.Height;

            depthFrameDesc = ks.DepthFrameSource.FrameDescription;

            uint colorFrameSize = colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels;
            uint depthFrameSize = depthFrameDesc.BytesPerPixel * depthFrameDesc.LengthInPixels;

            colorPixels = new byte[colorFrameSize];
            depthPixels = new byte[depthFrameSize];

            colorMappedToDepthPoints = new DepthSpacePoint[colorWidth * colorHeight];

            recordStarted = false;

            // Deleting all previous image in ./rgb directory
            System.IO.DirectoryInfo rgb_directory = new DirectoryInfo("./rgb/");
            foreach (FileInfo file in rgb_directory.GetFiles())
            {
                file.Delete();
            }
            // Deleting all previous image in ./d directory
            System.IO.DirectoryInfo d_directory = new DirectoryInfo("./d/");
            foreach (FileInfo file in d_directory.GetFiles())
            {
                file.Delete();
            }

            multiSourceFrameReader = ks.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);
            multiSourceFrameReader.MultiSourceFrameArrived += msfr_FrameArrived;

            start.Click += start_Click;
            stop.Click += stop_Click;
        }

        private void msfr_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {

            bool depthFrameProcessed = false;

            if (e.FrameReference == null) return;

            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();


                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if ( (colorFrame == null) || (depthFrame == null) )
                {
                    return;
                }

                // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                using (KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                {
                    coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(
                        depthBuffer.UnderlyingBuffer,
                        depthBuffer.Size,
                        this.colorMappedToDepthPoints);

                    // Note: In order to see the full range of depth (including the less reliable far field depth)
                    // we are setting maxDepth to the extreme potential depth threshold
                    ushort maxDepth = ushort.MaxValue;

                    // If you wish to filter by reliable depth distance, uncomment the following line:
                    //// maxDepth = depthFrame.DepthMaxReliableDistance

                    ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                    depthFrameProcessed = true;
                }


                colorFrame.CopyConvertedFrameDataToArray(colorPixels, ColorImageFormat.Bgra);
                //depthFrame.CopyFrameDataToArray(depthPixels);

                // Creating BitmapSource
                var bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel) / 8;
                var colorStride = bytesPerPixel * colorFrame.FrameDescription.Width;
                //var depthStride = bytesPerPixel * depthFrame.FrameDescription.Width;

                colorBitmapSource = BitmapSource.Create(colorFrameDesc.Width, colorFrameDesc.Height, 96.0, 96.0, PixelFormats.Bgr32, null, colorPixels, colorStride);
                //depthBitmapSource = BitmapSource.Create(depthFrameDesc.Width, depthFrameDesc.Height, 96.0, 96.0, PixelFormats.Gray8, null);

                // WriteableBitmap to show on UI
                writeableBitmap = new WriteableBitmap(this.depthFrameDesc.Width, this.depthFrameDesc.Height, 96.0, 96.0, PixelFormats.Gray8, null);
                kinectImage.Source = writeableBitmap;


                if (depthFrameProcessed)
                {
                    writeableBitmap.WritePixels(new Int32Rect(0, 0, writeableBitmap.PixelWidth, writeableBitmap.PixelHeight),
                        depthPixels,
                        writeableBitmap.PixelWidth,
                        0);
                }

                // We're done with the DepthFrame 
                depthFrame.Dispose();
                depthFrame = null;

                // We're done with the ColorFrame 
                colorFrame.Dispose();
                colorFrame = null;

            }
            finally
            {
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }
            }
        }


        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDesc.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        private void start_Click(object sender, RoutedEventArgs e)
        {
            imageSerial = 0;
            recordStarted = true;
        }

        private void stop_Click(object sender, RoutedEventArgs e)
        {
            recordStarted = false;
        }

    }
}
