// Copyright © 2017 Nico Stokman
//
// This component is used to read data from a Dht22 sensor.
//
// Usage:
// 
// This Dht22 sensor class is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Dht22 sensor class is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Dht22 sensor class. If not, see http://www.gnu.org/licenses/.
//

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using UwHelper;
using Windows.Devices.Gpio;

namespace Sensors.UW
{
    public class Dht22Sensor : IDisposable
    {
        // For specifications see:
        // - http://embedded-lab.com/blog/measurement-of-temperature-and-relative-humidity-using-dht11-sensor-and-pic-microcontroller/ 
        // - https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf
        //// Setting some constants 
        const int StartLowTimeInMicros = 5000;  // 1 .. 10 milli seconds for the start 
        const int StartHighTimeInMicros = 30;    // 20..40 microseconds for sending high as start signal to sensor
        // 
        // This is the threshold used to determine whether a bit is a '0' or a '1'.
        // For a '0' the pin will be high for 70 micro seconds
        // For a '1' the pin will be high for 26-28 micro seconds
        // So a threshold of 49 micro seconds is taken
        //
        const int OneBitHighTimeThresholdInMicros = 49;
        
        const int ReceivingDataBitsTimeoutMilliSeconds = 20;     // max 10 milli seconds for reading all of the data bits

        private GpioPin _dhtPin;
        private Stopwatch _stopwatch;
        private long _stopwatchFrequency = Stopwatch.Frequency;
        private List<Edge> _receivedSignal;
        string _errorMessage;

        private struct Edge
        {
            public int MicrosFromStart; // relative moment to start of input signal
            public GpioPinValue NewValue;  // either 1=high or 0=low
        }

        public Dht22Sensor(int pinNumber)
        {
            _dhtPin = GpioController.GetDefault().OpenPin(pinNumber);

            InitializeDataMeasurement();
        }

        private void InitializeDataMeasurement()
        {
            _dhtPin.SetDriveMode(GpioPinDriveMode.Input);

            _errorMessage = "";
            _receivedSignal = new List<Edge>();

            _stopwatch = new Stopwatch();
            _stopwatch.Start();
        }


        public DhtMeasurement GetMeasurement()
        {
            int maxRetries = 5;
            int retries = 0;
            DhtMeasurement measurement = new DhtMeasurement() { IsValid = false };

            while (retries < maxRetries)
            {
                InitializeDataMeasurement();

                SendStartToSensor();

                ReadSignalFromSensor();

                measurement = CalculateResult();

                if (measurement.IsValid)
                {
                    break;
                }
                else if (_receivedSignal.Count == 81)
                {
                    Edge edge = _receivedSignal.FirstOrDefault();
                    // missed just one edge, might be the first one.
                    Logging.LogMessage("Missed one edge from Dht22, first edge is {0} after micros: {1})", edge.NewValue.ToString(), edge.MicrosFromStart);
                }
                else if (retries > 0) // don't log first miss, happens more often
                {
                    Logging.LogMessage("Invalid Dht22 sensor read, message: {0} (retryCount: {1})", ErrorMessage, retries);
                }

                // wait 2 seconds, so that sensor is able to recover...
                Task.Delay(TimeSpan.FromSeconds(2)).Wait();

                retries++;
            }

            _dhtPin.SetDriveMode(GpioPinDriveMode.Input);

            return measurement;
        }

        private void ReadSignalFromSensor()
        {
            long startMeasurement = GetMicroSeconds();
            long endMeasurement = startMeasurement + ReceivingDataBitsTimeoutMilliSeconds * 1000;
            GpioPinValue currentValue = _dhtPin.Read();
            GpioPinValue newValue = currentValue;
            int receivedSignals = 0;
            long now = 0;

            do
            {
                now = GetMicroSeconds();
                newValue = _dhtPin.Read();

                if(newValue != currentValue)
                {
                    // changeDetected
                    _receivedSignal.Add(new Edge
                    {
                        MicrosFromStart = (int)(now - startMeasurement),
                        NewValue = newValue
                    });

                    currentValue = newValue;
                    receivedSignals++;
                }
            } while (now < endMeasurement && receivedSignals < 82) ;
            // just wait this 10 millis to receive the full data signal or stop when 82 changes are detected (1 ack bit and 40 data bits)
        }

        private void SendStartToSensor()
        {
            // Set pin as output and hold low (1..10..18 ms according specs)
            _dhtPin.SetDriveMode(GpioPinDriveMode.Output);

            _dhtPin.Write(GpioPinValue.Low);
            DelayMicroseconds(StartLowTimeInMicros);

            // high for 30 micro seconds (send the start signal
            _dhtPin.Write(GpioPinValue.High);
            DelayMicroseconds(StartHighTimeInMicros);

            _dhtPin.SetDriveMode(GpioPinDriveMode.Input);
        }


        private DhtMeasurement CalculateResult()
        {
            byte[] bits = new byte[40];
            int[] bytes = new int[5];

            string bitStream = "";
            DhtMeasurement measurement = new DhtMeasurement();

            // we expect at least 1 raise and fall for the ack-bit and and 40 raise-fall combinations
            if (_receivedSignal.Count >= 2 + 2 * 40)
            {
                Edge[] signals = _receivedSignal.Take(82).ToArray();

                for (int bitNr = 0; bitNr < 40; bitNr++)
                {
                    // substract the raise-time from the fall-time 
                    int highInMicros = signals[2 + bitNr * 2 + 1].MicrosFromStart - signals[2 + bitNr * 2].MicrosFromStart;
                    bits[bitNr] = (highInMicros < OneBitHighTimeThresholdInMicros) ? (byte)0 : (byte)1;
                    bitStream += bits[bitNr].ToString() + "|";

                    int byteNr = bitNr / 8;
                    bytes[byteNr] = (bytes[byteNr] << 1) + bits[bitNr];
                }

                int checkSum = (bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xff;
                if (checkSum != bytes[4])
                {
                    measurement.IsValid = false;
                    _errorMessage = string.Format("Checksum invalid. Expected: {0}, Calculated: {1}", bytes[4], checkSum);
                    return measurement;
                }

                measurement.Humidity = (double)(256 * bytes[0] + bytes[1]) / 10;
                measurement.Temperature = (double)((bytes[2] & 0x7f) * 256 + bytes[3]) / 10;
                if ((bytes[2] & 0x80) > 0) { measurement.Temperature *= -1; }  // if first bit of MSB temperature then temp is negative

                measurement.IsValid = true;
            }
            else
            {
                _errorMessage = String.Format("Only {0} signal changes registered, 82 expected.", _receivedSignal.Count);
            }

            return measurement;
        }

        public string ErrorMessage
        {
            get { return _errorMessage;  }
        }

        public object AdditionalInfo { get; set; }


        private Int64 GetMicroSeconds()
        {
            // 10 ticks in a micro second: https://msdn.microsoft.com/en-us/library/system.datetime.ticks%28v=vs.110%29.aspx
            // DateTime.Now is not updated frequently enough
            return _stopwatch.ElapsedTicks * 1000000 / _stopwatchFrequency;
        }

        private void DelayMicroseconds(short periodInMicroSeconds)
        {
            // 10 ticks per micro seconds fixed for TimeSpan
            long start = GetMicroSeconds();
            while (GetMicroSeconds() - start < periodInMicroSeconds) { }

            //await Task.Delay(TimeSpan.FromTicks(periodInMicroSeconds * 10));
        }


        public void Dispose()
        {
            if (_stopwatch != null)
            {
                _stopwatch.Stop();
            }
            if (_dhtPin != null) { _dhtPin.Dispose(); }
        }
    }
}
