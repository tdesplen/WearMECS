//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace WearMEBenchTopControlSystem
{
    class LoggerThread
    {

        public StreamWriter writer;
        public List<List<double>> data;

        //should use threads to write data to file and to the interface
        public int WriteToFile()
        {
            string line = "";
            for (int j = 0; j < data[0].Count; j++)
            {
                line = "";
                for (int i = 0; i < data.Count; i++)
                {
                    //if (i != data.Count - 1)
                    //    line += data[i][j].ToString("0." + new string('#',11)) + ",";
                    //else
                    //    line += data[i][j].ToString("0." + new string('#', 11));
                    if (i != data.Count - 1)
                        line += data[i][j] + ",";
                    else
                        line += data[i][j];
                }
                writer.WriteLine(line);
            }

            return data[0].Count;
        }
    }
}
