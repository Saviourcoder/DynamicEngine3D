/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEngine;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
#if UNITY_EDITOR
    [System.Serializable]
    public class TrussJson
    {
        [System.Serializable]
        public class JsonVector3
        {
            public float x, y, z;

            public Vector3 ToVector3() => new Vector3(x, y, z);
            public static JsonVector3 FromVector3(Vector3 v) => new JsonVector3 { x = v.x, y = v.y, z = v.z };
        }

        [System.Serializable]
        public class JsonTrussBeam
        {
            public int nodeA;
            public int nodeB;
            public float compliance;
            public float damping;
            public float restLength;
            public float breakingThreshold;
            public float maxDeformation = 0.5f;
            public float plasticityThreshold = 0.05f;
            public float plasticityRate = 0.1f;
            public bool isActive = true;

            public Beam ToTrussBeam() => new Beam(
                nodeA: nodeA,
                nodeB: nodeB,
                compliance: compliance,
                damping: damping,
                restLength: restLength,
                maxDeformation: maxDeformation,
                plasticityThreshold: plasticityThreshold,
                plasticityRate: plasticityRate
            );
        }

        [System.Serializable]
        public class JsonNamedSet
        {
            public string name;
            public int[] indices;
            public Color color;
            public bool isVisible;
        }

        [System.Serializable]
        public class JsonFace
        {
            public int nodeA;
            public int nodeB;
            public int nodeC;

            public Face ToFace() => new Face(nodeA, nodeB, nodeC);
        }

        [System.Serializable]
        public class JsonBeamProperties
        {
            public string name;
            public float compliance;
            public float damping;

            public static JsonBeamProperties FromTruss(Truss truss) => new JsonBeamProperties
            {
                name = truss.name,
                compliance = truss.compliance,
                damping = truss.DefaultBeamDamping
            };
        }

        public JsonVector3[] nodePositions;
        public float[] nodeMasses;
        public JsonTrussBeam[] beams;
        public JsonNamedSet[] nodeSets;
        public JsonNamedSet[] linkSets;
        public JsonFace[] faces;
        public JsonBeamProperties beamProperties;
        public float maxStretchFactor = 1.05f;
        public float minStretchFactor = 0.95f;
        public int[] pinnedNodes;
        public JsonVector3 centerOfMass;

        public static TrussJson FromTxTruss(TxTruss txTruss)
        {
            if (txTruss == null)
            {
                Debug.LogError("TxTruss is null in FromTxTruss");
                return new TrussJson();
            }

            var json = new TrussJson
            {
                nodePositions = new JsonVector3[txTruss.nodeCount],
                nodeMasses = txTruss.nodeMass != null ? (float[])txTruss.nodeMass.Clone() : Enumerable.Repeat(0.5f, txTruss.nodeCount).ToArray(),
                beams = new JsonTrussBeam[txTruss.linkCount],
                nodeSets = new JsonNamedSet[txTruss.nodesSet != null ? txTruss.nodesSet.Length : 0],
                linkSets = new JsonNamedSet[txTruss.linksSet != null ? txTruss.linksSet.Length : 0],
                faces = new JsonFace[txTruss.faceCount],
                beamProperties = new JsonBeamProperties
                {
                    name = txTruss.name,
                    compliance = 0.01f,
                    damping = 0.3f
                },
                maxStretchFactor = 1.05f,
                minStretchFactor = 0.95f,
                pinnedNodes = new int[0],
                centerOfMass = JsonVector3.FromVector3(Vector3.zero)
            };

            // Populate node positions
            for (int i = 0; i < txTruss.nodeCount; i++)
            {
                json.nodePositions[i] = JsonVector3.FromVector3(txTruss.nodePosition != null && i < txTruss.nodePosition.Length ? txTruss.nodePosition[i] : Vector3.zero);
            }

            // Populate beams
            for (int i = 0; i < txTruss.linkCount; i++)
            {
                json.beams[i] = new JsonTrussBeam
                {
                    nodeA = txTruss.linkNodes != null && i * 2 < txTruss.linkNodes.Length ? txTruss.linkNodes[i * 2] : 0,
                    nodeB = txTruss.linkNodes != null && i * 2 + 1 < txTruss.linkNodes.Length ? txTruss.linkNodes[i * 2 + 1] : 0,
                    compliance = txTruss.linkStiffness != null && i < txTruss.linkStiffness.Length ? (txTruss.linkStiffness[i] > 0 ? 1f / txTruss.linkStiffness[i] : 0.01f) : 0.01f,
                    damping = txTruss.linkDamping != null && i < txTruss.linkDamping.Length ? txTruss.linkDamping[i] : 0.3f,
                    restLength = txTruss.linkLength != null && i < txTruss.linkLength.Length ? txTruss.linkLength[i] : 0f,
                    breakingThreshold = txTruss.linkBreaking != null && i < txTruss.linkBreaking.Length ? txTruss.linkBreaking[i] : 0f,
                    maxDeformation = 0.5f,
                    plasticityThreshold = 0.05f,
                    plasticityRate = 0.1f,
                    isActive = txTruss.linkFlags != null && i < txTruss.linkFlags.Length ? (txTruss.linkFlags[i] != 0) : true
                };
            }

            // Populate node sets
            for (int i = 0; i < (txTruss.nodesSet != null ? txTruss.nodesSet.Length : 0); i++)
            {
                json.nodeSets[i] = new JsonNamedSet
                {
                    name = txTruss.nodesSet[i]?.name ?? ("NodeSet" + i),
                    indices = txTruss.nodesSet[i]?.indices?.ToArray() ?? new int[0],
                    color = Color.cyan,
                    isVisible = true
                };
            }

            // Populate link sets
            for (int i = 0; i < (txTruss.linksSet != null ? txTruss.linksSet.Length : 0); i++)
            {
                json.linkSets[i] = new JsonNamedSet
                {
                    name = txTruss.linksSet[i]?.name ?? ("LinkSet" + i),
                    indices = txTruss.linksSet[i]?.indices?.ToArray() ?? new int[0],
                    color = Color.magenta,
                    isVisible = true
                };
            }

            // Populate faces
            for (int i = 0; i < txTruss.faceCount; i++)
            {
                json.faces[i] = new JsonFace
                {
                    nodeA = txTruss.faceNodes != null && i * 3 < txTruss.faceNodes.Length ? txTruss.faceNodes[i * 3] : 0,
                    nodeB = txTruss.faceNodes != null && i * 3 + 1 < txTruss.faceNodes.Length ? txTruss.faceNodes[i * 3 + 1] : 0,
                    nodeC = txTruss.faceNodes != null && i * 3 + 2 < txTruss.faceNodes.Length ? txTruss.faceNodes[i * 3 + 2] : 0
                };
            }

            return json;
        }

        public Truss ToTruss()
        {
            Truss truss = ScriptableObject.CreateInstance<Truss>();
            truss.SetNodePositions(nodePositions?.Select(v => v.ToVector3()).ToArray() ?? new Vector3[0]);
            truss.SetBeams(beams?.Select(b => b.ToTrussBeam()).ToList() ?? new List<Beam>());
            truss.SetPinnedNodes(pinnedNodes?.ToList() ?? new List<int>());
            truss.SetNodeMasses(nodeMasses?.ToList() ?? new List<float>());
            truss.SetPhysicsProperties(nodeMasses != null && nodeMasses.Length > 0 ? nodeMasses[0] : 0.5f, maxStretchFactor, minStretchFactor);

            // Set node sets
            truss.SetNodeSets(nodeSets?.Select(ns => new NodeSet(ns.name, ns.indices.ToList())
            {
                color = ns.color,
                isVisible = ns.isVisible
            }).ToList() ?? new List<NodeSet>());

            // Set link sets
            truss.SetLinkSets(linkSets?.Select(ls => new LinkSet(ls.name, ls.indices.ToList())
            {
                color = ls.color,
                isVisible = ls.isVisible
            }).ToList() ?? new List<LinkSet>());

            // Add faces
            foreach (var face in faces ?? new JsonFace[0])
            {
                truss.AddFace(face.ToFace());
            }

            return truss;
        }
    }

    [InitializeOnLoad]
    public static class Converter
    {
        [MenuItem("Assets/Convert TxTruss to JSON", false, 101)]
        private static void ConvertTxTrussToJson()
        {
            UnityEngine.Object[] selectedAssets = Selection.objects;
            foreach (UnityEngine.Object asset in selectedAssets)
            {
                if (asset is TxTruss txTruss)
                {
                    string path = EditorUtility.SaveFilePanel("Export TxTruss to JSON", "", asset.name + ".json", "json");
                    if (!string.IsNullOrEmpty(path))
                    {
                        TrussJson jsonData = TrussJson.FromTxTruss(txTruss);
                        string json = JsonUtility.ToJson(jsonData, true);
                        File.WriteAllText(path, json);
                        Debug.Log($"Converted TxTruss '{txTruss.name}' to JSON at '{path}'", txTruss);

                        if (jsonData.beams != null && jsonData.beams.Length > 0)
                        {
                            Debug.Log($"Exported {jsonData.nodePositions.Length} nodes, {jsonData.beams.Length} beams, " +
                                     $"{jsonData.nodeSets?.Length ?? 0} node sets, {jsonData.linkSets?.Length ?? 0} link sets", txTruss);
                        }
                    }
                }
            }
            AssetDatabase.Refresh();
        }

        [MenuItem("Assets/Convert TxTruss to JSON", true)]
        private static bool ValidateConvertTxTrussToJson()
        {
            foreach (UnityEngine.Object asset in Selection.objects)
            {
                if (asset is TxTruss)
                {
                    return true;
                }
            }
            return false;
        }

        [MenuItem("Assets/Import Truss from JSON", false, 102)]
        private static void ImportTrussFromJson()
        {
            string path = EditorUtility.OpenFilePanelWithFilters("Select Truss JSON", "", new[] { "JSON files", "json" });
            if (string.IsNullOrEmpty(path))
            {
                Debug.Log("Import cancelled: No file selected.");
                return;
            }

            if (!File.Exists(path))
            {
                Debug.LogError($"JSON file not found: {path}");
                return;
            }

            try
            {
                string json = File.ReadAllText(path);
                TrussJson trussJson = JsonUtility.FromJson<TrussJson>(json);
                if (trussJson == null)
                {
                    Debug.LogError($"Failed to parse JSON: {path}");
                    return;
                }

                Truss truss = trussJson.ToTruss();
                string assetPath = "Assets/" + System.IO.Path.GetFileNameWithoutExtension(path) + ".asset";
                assetPath = AssetDatabase.GenerateUniqueAssetPath(assetPath);

                AssetDatabase.CreateAsset(truss, assetPath);
                Debug.Log($"Imported Truss from JSON at '{path}' to '{assetPath}'", truss);

                if (truss.GetTrussBeams().Count > 0)
                {
                    Debug.Log($"Imported {truss.NodePositions.Length} nodes, {truss.GetTrussBeams().Count} beams, " +
                              $"{truss.GetNodeSets().Count} node sets, {truss.GetLinkSets().Count} link sets, " +
                              $"First beam: compliance={truss.GetTrussBeams()[0].compliance:F6}, damping={truss.GetTrussBeams()[0].damping:F3}", truss);
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Error importing JSON '{path}': {e.Message}");
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
        }
    }
#endif
}