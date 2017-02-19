using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sensors.UW
{
    public struct DhtMeasurement
    {
        public bool IsValid;

        public double Humidity;
        public double Temperature;
    }
}
