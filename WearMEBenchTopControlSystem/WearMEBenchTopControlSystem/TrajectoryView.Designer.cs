//© 2019. Tyler Desplenter and Ana Luisa Trejos.

namespace WearMEBenchTopControlSystem
{
    partial class TrajectoryView
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series2 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series3 = new System.Windows.Forms.DataVisualization.Charting.Series();
            this.TrajectoryGraph = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.subject_elbow_angle = new System.Windows.Forms.TextBox();
            this.brace_elbow_angle = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.TrajectoryGraph)).BeginInit();
            this.SuspendLayout();
            // 
            // TrajectoryGraph
            // 
            chartArea1.AxisX.LabelAutoFitMaxFontSize = 14;
            chartArea1.AxisX.Maximum = 500D;
            chartArea1.AxisX.Minimum = 0D;
            chartArea1.AxisX.TitleFont = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            chartArea1.AxisY.Interval = 20D;
            chartArea1.AxisY.IsStartedFromZero = false;
            chartArea1.AxisY.LabelAutoFitMaxFontSize = 14;
            chartArea1.AxisY.Maximum = 140D;
            chartArea1.AxisY.Minimum = -20D;
            chartArea1.Name = "ChartArea1";
            this.TrajectoryGraph.ChartAreas.Add(chartArea1);
            legend1.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            legend1.IsTextAutoFit = false;
            legend1.Name = "Legend1";
            this.TrajectoryGraph.Legends.Add(legend1);
            this.TrajectoryGraph.Location = new System.Drawing.Point(231, 22);
            this.TrajectoryGraph.Name = "TrajectoryGraph";
            series1.ChartArea = "ChartArea1";
            series1.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
            series1.Legend = "Legend1";
            series1.Name = "DesiredTrajectory";
            series1.YValuesPerPoint = 2;
            series2.ChartArea = "ChartArea1";
            series2.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
            series2.Legend = "Legend1";
            series2.Name = "SubjectTrajectory";
            series3.ChartArea = "ChartArea1";
            series3.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
            series3.Legend = "Legend1";
            series3.Name = "BraceTrajectory";
            series3.YValuesPerPoint = 2;
            this.TrajectoryGraph.Series.Add(series1);
            this.TrajectoryGraph.Series.Add(series2);
            this.TrajectoryGraph.Series.Add(series3);
            this.TrajectoryGraph.Size = new System.Drawing.Size(1232, 652);
            this.TrajectoryGraph.TabIndex = 0;
            this.TrajectoryGraph.Text = "chart1";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(21, 57);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(205, 24);
            this.label1.TabIndex = 1;
            this.label1.Text = "Subect Elbow Angle (°)";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(31, 324);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(195, 24);
            this.label2.TabIndex = 2;
            this.label2.Text = "Brace Elbow Angle (°)";
            // 
            // subject_elbow_angle
            // 
            this.subject_elbow_angle.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.subject_elbow_angle.Location = new System.Drawing.Point(71, 84);
            this.subject_elbow_angle.Name = "subject_elbow_angle";
            this.subject_elbow_angle.ReadOnly = true;
            this.subject_elbow_angle.Size = new System.Drawing.Size(110, 29);
            this.subject_elbow_angle.TabIndex = 3;
            // 
            // brace_elbow_angle
            // 
            this.brace_elbow_angle.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.brace_elbow_angle.Location = new System.Drawing.Point(71, 360);
            this.brace_elbow_angle.Name = "brace_elbow_angle";
            this.brace_elbow_angle.ReadOnly = true;
            this.brace_elbow_angle.Size = new System.Drawing.Size(110, 29);
            this.brace_elbow_angle.TabIndex = 4;
            // 
            // TrajectoryView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1475, 695);
            this.Controls.Add(this.brace_elbow_angle);
            this.Controls.Add(this.subject_elbow_angle);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.TrajectoryGraph);
            this.Name = "TrajectoryView";
            this.Text = "TrajectoryView";
            ((System.ComponentModel.ISupportInitialize)(this.TrajectoryGraph)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.DataVisualization.Charting.Chart TrajectoryGraph;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox subject_elbow_angle;
        private System.Windows.Forms.TextBox brace_elbow_angle;
    }
}