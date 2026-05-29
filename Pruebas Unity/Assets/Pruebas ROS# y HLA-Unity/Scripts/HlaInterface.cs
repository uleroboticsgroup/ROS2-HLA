using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace HLA
{
    /// <summary>
    /// Role of the federate for time management priority
    /// </summary>
    public enum FederateRole
    {
        Sender = 0,
        Viewer = 1,
        Hybrid = 2
    }

    /// <summary>
    /// Data structure for Box position (existing)
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
    /// Time Management state information from HLA RTI
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

    public static class HlaInterface
    {
        private const string DllName = "hla_plugin";

        // === Connection & Object Management (existing) ===
        
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Connect([MarshalAs(UnmanagedType.LPStr)] string federationName, [MarshalAs(UnmanagedType.LPStr)] string federateName, [MarshalAs(UnmanagedType.LPStr)] string fomFilePath);

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

        // === Time Management API (NEW) ===

        /// <summary>
        /// Enable HLA Time Regulating and Time Constrained for this federate.
        /// </summary>
        /// <param name="role">Federate role (Sender=0, Viewer=1, Hybrid=2)</param>
        /// <param name="lookahead">Lookahead value in seconds (lower = higher priority)</param>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void EnableTimeManagement(int role, double lookahead);

        /// <summary>
        /// Disable HLA Time Management for this federate.
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DisableTimeManagement();

        /// <summary>
        /// Request advancement of logical time by timeStep seconds.
        /// </summary>
        /// <param name="timeStep">Time increment in seconds</param>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void RequestTimeAdvance(double timeStep);

        /// <summary>
        /// Check if the last time advance request was granted.
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IsTimeAdvanceGranted();

        /// <summary>
        /// Get the current granted logical time.
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double GetCurrentLogicalTime();

        /// <summary>
        /// Get the full time management state.
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern TimeManagementState GetTimeManagementState();

        /// <summary>
        /// Process RTI callbacks for up to 'seconds' duration.
        /// </summary>
        /// <param name="seconds">Maximum time to process callbacks</param>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void EvokeCallbacks(double seconds);
    }
}
