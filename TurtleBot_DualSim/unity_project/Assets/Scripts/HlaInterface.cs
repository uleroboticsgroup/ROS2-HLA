using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace HLA
{
    /// <summary>
    /// Roles del federado para gestión de tiempo HLA
    /// </summary>
    public enum FederateRole
    {
        Sender = 0,
        Viewer = 1,
        Hybrid = 2
    }

    /// <summary>
    /// Estructura de datos compatible con el plugin C++ de HLA.
    /// Cada robot (local o remoto) se representa con esta estructura.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BoxData
    {
        public int id;
        public float positionX;
        public float positionY;
        public float rotationY;
        public float positionZ;
    }

    /// <summary>
    /// Estado de gestión de tiempo HLA
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct TimeManagementState
    {
        [MarshalAs(UnmanagedType.I1)]
        public bool isTimeRegulating;
        [MarshalAs(UnmanagedType.I1)]
        public bool isTimeConstrained;
        [MarshalAs(UnmanagedType.I1)]
        public bool timeAdvanceGranted;
        public double currentLogicalTime;
        public double lookahead;
    }

    /// <summary>
    /// Interfaz P/Invoke al plugin C++ de HLA (hla_plugin.so / hla_plugin.dll)
    /// </summary>
    public static class HlaInterface
    {
        private const string DllName = "hla_plugin";

        // === Conexión y gestión de objetos ===

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Connect(
            [MarshalAs(UnmanagedType.LPStr)] string federationName,
            [MarshalAs(UnmanagedType.LPStr)] string federateName,
            [MarshalAs(UnmanagedType.LPStr)] string fomFilePath);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Disconnect();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void PublishUnit();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SubscribeUnit();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int CreateUnit();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void UpdateUnit(BoxData objData);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetBoxes(out int count);

        // === Gestión de Tiempo ===

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void EnableTimeManagement(int role, double lookahead);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DisableTimeManagement();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void RequestTimeAdvance(double timeStep);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IsTimeAdvanceGranted();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double GetCurrentLogicalTime();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern TimeManagementState GetTimeManagementState();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void EvokeCallbacks(double seconds);
    }
}
