using System;
using UnityEngine;

public class TxTruss : ScriptableObject
{
	[Serializable]
	public class NamedSet
	{
		public string name = "";

		public int[] indices = new int[0];
	}

	public Vector3[] nodePosition = new Vector3[0];

	public float[] nodeMass = new float[0];

	public NamedSet[] nodesSet = new NamedSet[0];

	public int[] linkNodes = new int[0];

	public float[] linkLength = new float[0];

	public float[] linkStiffness = new float[0];

	public float[] linkDamping = new float[0];

	public float[] linkElastic = new float[0];

	public float[] linkBreaking = new float[0];

	public float[] linkStretching = new float[0];

	public int[] linkFlags = new int[0];

	public NamedSet[] linksSet = new NamedSet[0];

	public int[] faceNodes = new int[0];

	public int[] faceFlags = new int[0];

	public int[] faceMatter = new int[0];

	public float[] faceEnvelope = new float[0];

	public NamedSet[] facesSet = new NamedSet[0];

	public int nodeCount
	{
		get
		{
			return nodePosition.Length;
		}
		set
		{
			Array.Resize(ref nodePosition, value);
			Array.Resize(ref nodeMass, value);
		}
	}

	public int linkCount
	{
		get
		{
			return linkNodes.Length / 2;
		}
		set
		{
			Array.Resize(ref linkNodes, value * 2);
			Array.Resize(ref linkLength, value);
			Array.Resize(ref linkStiffness, value);
			Array.Resize(ref linkDamping, value);
			Array.Resize(ref linkElastic, value);
			Array.Resize(ref linkBreaking, value);
			Array.Resize(ref linkStretching, value);
			Array.Resize(ref linkFlags, value);
		}
	}

	public int faceCount
	{
		get
		{
			return faceNodes.Length / 3;
		}
		set
		{
			Array.Resize(ref faceNodes, value * 3);
			Array.Resize(ref faceFlags, value);
			Array.Resize(ref faceMatter, value);
			Array.Resize(ref faceEnvelope, value);
		}
	}

	public int[] FindNodeSet(string _name)
	{
		return Array.Find(nodesSet, (NamedSet x) => x.name == _name)?.indices;
	}

	public int[] FindLinkSet(string _name)
	{
		return Array.Find(linksSet, (NamedSet x) => x.name == _name)?.indices;
	}

	public int[] FindFaceSet(string _name)
	{
		return Array.Find(facesSet, (NamedSet x) => x.name == _name)?.indices;
	}
}
