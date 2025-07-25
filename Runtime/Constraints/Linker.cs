/* DynamicEngine3D - Node Set Helper
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace DynamicEngine
{
    public static class NodeSetHelper
    {
        public static int[] FindNodeSet(SoftBody softBody, string nodeSetName)
        {
            if (softBody?.truss == null || string.IsNullOrEmpty(nodeSetName)) 
                return null;
            
            // First check for named node sets from NodeLinkEditor
            var nodeLinkEditor = softBody.GetComponent<NodeLinkEditor>();
            if (nodeLinkEditor != null && nodeLinkEditor.nodeSets != null)
            {
                var namedNodeSet = nodeLinkEditor.nodeSets.FirstOrDefault(ns => ns.name == nodeSetName);
                if (namedNodeSet != null && namedNodeSet.IsValid())
                {
                    // Validate indices are still valid
                    var validIndices = namedNodeSet.nodeIndices
                        .Where(i => softBody.solver?.nodeManager?.Nodes != null && 
                                   i >= 0 && i < softBody.solver.nodeManager.Nodes.Count)
                        .ToArray();
                    return validIndices.Length > 0 ? validIndices : null;
                }
            }
            
            // This integrates with your existing truss system
            // You would implement this based on how your truss system works
            // For now, here's a basic implementation for node naming patterns:
            
            // Handle single node reference: "node0", "node1", etc.
            if (nodeSetName.StartsWith("node"))
            {
                if (int.TryParse(nodeSetName.Substring(4), out int index))
                {
                    if (softBody.solver?.nodeManager?.Nodes != null && 
                        index >= 0 && index < softBody.solver.nodeManager.Nodes.Count)
                    {
                        return new int[] { index };
                    }
                }
            }
            
            // Handle range references: "nodes0-3" (nodes 0, 1, 2, 3)
            if (nodeSetName.StartsWith("nodes") && nodeSetName.Contains("-"))
            {
                string rangeStr = nodeSetName.Substring(5);
                string[] parts = rangeStr.Split('-');
                if (parts.Length == 2 && int.TryParse(parts[0], out int start) && int.TryParse(parts[1], out int end))
                {
                    var result = new List<int>();
                    for (int i = start; i <= end; i++)
                    {
                        if (softBody.solver?.nodeManager?.Nodes != null && 
                            i >= 0 && i < softBody.solver.nodeManager.Nodes.Count)
                        {
                            result.Add(i);
                        }
                    }
                    return result.ToArray();
                }
            }
            
            // Handle comma-separated list: "nodes0,2,5"
            if (nodeSetName.StartsWith("nodes") && nodeSetName.Contains(","))
            {
                string listStr = nodeSetName.Substring(5);
                string[] parts = listStr.Split(',');
                var result = new List<int>();
                foreach (string part in parts)
                {
                    if (int.TryParse(part.Trim(), out int index))
                    {
                        if (softBody.solver?.nodeManager?.Nodes != null && 
                            index >= 0 && index < softBody.solver.nodeManager.Nodes.Count)
                        {
                            result.Add(index);
                        }
                    }
                }
                return result.ToArray();
            }
            
            // Handle named groups - this would integrate with your truss system
            // Example: "corner_nodes", "edge_nodes", etc.
            // You would implement this based on your truss naming system
            
            return null;
        }
    }
}