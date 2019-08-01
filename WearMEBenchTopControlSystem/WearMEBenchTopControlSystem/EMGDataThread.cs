//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    class EMGDataThread
    {
        public DelsysTrignoCSharpLibrary.Driver trigno;
        public int window_size;
        public int biceps_emg_channel;
        public int triceps_emg_channel;

        //Thread Function
        public List<List<double>> GetEMGData()
        {
            List<List<double>> emg_data = new List<List<double>>();
            List<double> biceps_emg = new List<double>(window_size);
            List<double> triceps_emg = new List<double>(window_size);
            emg_data.Add(biceps_emg);
            emg_data.Add(triceps_emg);

            for (int i = 0; i < window_size; i++)
            {
                DelsysTrignoCSharpLibrary.EMGSample emg_sample = trigno.GetEMGSample();
                emg_data[0].Add(Convert.ToDouble(emg_sample.emgData[biceps_emg_channel]));
                emg_data[1].Add(Convert.ToDouble(emg_sample.emgData[triceps_emg_channel]));
            }

            return emg_data;
        }
    }
}
