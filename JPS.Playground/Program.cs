/*
 * Program.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian. MIT License.
 */

namespace JPS
{
    internal static class Program
    {
        /// <summary>WinForms 演示程序入口。</summary>
        [STAThread]
        static void Main()
        {
            ApplicationConfiguration.Initialize();
            Application.Run(new Form1());
        }
    }
}
