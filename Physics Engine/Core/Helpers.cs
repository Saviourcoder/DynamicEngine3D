/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
using UnityEngine;
using System.Collections.Generic;

namespace DynamicEngine
{
    public class SolverComponent : MonoBehaviour
    {
        public Solver solver;
    }
    public struct CollisionForce
    {
        public int nodeIndex;
        public Vector3 force;
        public Vector3 contactPoint;

        public CollisionForce(int nodeIndex, Vector3 force, Vector3 contactPoint)
        {
            this.nodeIndex = nodeIndex;
            this.force = force;
            this.contactPoint = contactPoint;
        }
    }

    public class SoftbodyCollisionManager
    {
        private static readonly Dictionary<int, List<CollisionForce>> pendingCollisionForces =
            new Dictionary<int, List<CollisionForce>>();
        private static readonly HashSet<int> activeSolvers = new HashSet<int>();

        public static void RegisterSolver(int solverID)
        {
            activeSolvers.Add(solverID);
            if (!pendingCollisionForces.ContainsKey(solverID))
                pendingCollisionForces[solverID] = new List<CollisionForce>();
        }

        public static void UnregisterSolver(int solverID)
        {
            activeSolvers.Remove(solverID);
            pendingCollisionForces.Remove(solverID);
        }

        public static void AddCollisionForce(int targetSolverID, CollisionForce force)
        {
            if (pendingCollisionForces.ContainsKey(targetSolverID))
            {
                pendingCollisionForces[targetSolverID].Add(force);
            }
        }

        public static List<CollisionForce> GetAndClearCollisionForces(int solverID)
        {
            if (!pendingCollisionForces.ContainsKey(solverID))
                return new List<CollisionForce>();

            var forces = new List<CollisionForce>(pendingCollisionForces[solverID]);
            pendingCollisionForces[solverID].Clear();
            return forces;
        }

        public static bool AllSolversFinishedDetection()
        {
            return activeSolvers.Count > 0;
        }
    }
}
