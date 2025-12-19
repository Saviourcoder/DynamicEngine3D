using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Mathematics;
using Vella.Common;
using UnityEngine;

namespace Vella.UnityNativeHull
{


    public unsafe struct NativeManifold : IDisposable
    {
        public const int MaxPoints = 24;
        public float3 Normal;

        private int _maxIndex;
        private NativeBuffer _points;

        public ContactPoint[] ToArray() => _points.ToArray<ContactPoint>(Length);
        public ContactPoint this[int i] => _points.GetItem<ContactPoint>(i);
        public int Length => _maxIndex + 1;
        public bool IsCreated => _points.IsCreated;

        public NativeManifold(Allocator allocator)
        {
            _points = NativeBuffer.Create<ContactPoint>(MaxPoints, Allocator.Persistent);
            _maxIndex = -1;
            Normal = 0;
        }

        public void Add(float3 position, float distance, ContactID id)
        {
            Add(new ContactPoint
            {
                Id = id,
                Position = position,
                Distance = distance,
            });
        }

        public void Add(ContactPoint cp)
        {
            if (_maxIndex + 1 >= MaxPoints)
            {
                Debug.LogWarning($"NativeManifold: Exceeded MaxPoints ({MaxPoints}), ignoring additional contact points");
                return;
            }

            _points.SetItem(++_maxIndex, cp);
        }

        public void Dispose()
        {
            if (_points.IsCreated)
            {
                _points.Dispose();
            }
        }
    }

    public struct ContactPoint
    {
        public ContactID Id;
        public float3 Position;
        public float NormalImpulse;
        public float3 Tangent1;
        public float3 Tangent2;
        public float TangentImpulse1;
        public float TangentImpulse2;
        public float Distance;
        public float3 Penetration;
        public float3 PositionOnTarget;
        public float3 PositionOnSource;
    }

    //[StructLayout(LayoutKind.Explicit)]
    public struct ContactID
    {
        //[FieldOffset(0)]
        public FeaturePair FeaturePair;
        //[FieldOffset(0)]
        public int Key;
    }

    public struct FeaturePair
    {
        public int InEdge1;
        public int OutEdge1;
        public int InEdge2;
        public int OutEdge2;
    }

}
